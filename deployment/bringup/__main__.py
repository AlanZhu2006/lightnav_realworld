"""Manage the observation/ImageGoal stack under work without root services.

Only processes started by this manager can be stopped. Healthy existing model
and camera processes are observed/adopted without taking lifecycle ownership.
No component in the shipped profiles imports a robot motion SDK.
"""
from __future__ import annotations
import argparse
from datetime import datetime,timezone
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from urllib.request import urlopen,Request
from urllib.parse import urlsplit


PROJECT=Path(__file__).resolve().parents[2]


def read_json(path):
    return json.loads(path.read_text()) if path.exists() else {}


def atomic(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(path)


def expand(value,run):
    if isinstance(value,str): return value.replace('{project}',str(PROJECT)).replace('{run}',str(run))
    if isinstance(value,list): return [expand(v,run) for v in value]
    if isinstance(value,dict): return {k:expand(v,run) for k,v in value.items()}
    return value


def proc_token(pid):
    try:
        values=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()
        return None if values[0]=='Z' else values[19]
    except (OSError,IndexError): return None


def owned_alive(item):
    stored=item.get('proc_start_token')
    return bool(item.get('ownership')=='managed' and stored is not None and proc_token(item['pid'])==stored)


def probe(component):
    try:
        if 'health_url' in component:
            with urlopen(component['health_url'],timeout=2) as stream: value=json.load(stream)
            for key,expected in component.get('health_expect',{}).items():
                actual=value
                for part in key.split('.'): actual=actual.get(part) if isinstance(actual,dict) else None
                if actual!=expected: return False,f'{key}={actual!r}'
            return True,'HTTP health contract passed'
        if 'health_command' in component:
            result=subprocess.run(component['health_command'],cwd=PROJECT,env={**os.environ,**component.get('env',{})},
                                  capture_output=True,text=True,timeout=component.get('probe_timeout_s',8))
            return result.returncode==0,(result.stdout+result.stderr)[-1800:]
        if 'tcp' in component:
            host,port=component['tcp']
            with socket.create_connection((host,port),timeout=1): pass
            return True,'TCP endpoint reachable; inspect protocol receipt for capability verification'
        return False,'no independent health probe'
    except Exception as exc:
        return False,f'{type(exc).__name__}: {exc}'


def unhealthy_http_port_occupied(component):
    if 'health_url' not in component: return False
    value=urlsplit(component['health_url'])
    try:
        with socket.create_connection((value.hostname,value.port or (443 if value.scheme=='https' else 80)),timeout=.5): pass
        return True
    except OSError: return False


def stop_owned(item):
    if not owned_alive(item): return
    try: os.killpg(item['pid'],signal.SIGTERM)
    except ProcessLookupError: return
    deadline=time.monotonic()+5
    while owned_alive(item) and time.monotonic()<deadline: time.sleep(.1)
    if owned_alive(item):
        try: os.killpg(item['pid'],signal.SIGKILL)
        except ProcessLookupError: pass


def cancel_task(state):
    item=next((v for v in state.get('components',[]) if v['name']=='runtime'),None)
    if not item or not owned_alive(item) or not item.get('runtime_session_id'): return
    url=state.get('runtime_url','http://127.0.0.1:8093')
    try:
        with urlopen(url.rstrip('/')+'/status',timeout=2) as stream: live=json.load(stream)
        if live.get('session_id')!=item['runtime_session_id']: return
        req=Request(url.rstrip('/')+'/task/cancel',data=b'{"reason":"stack_stop"}',headers={'Content-Type':'application/json'})
        with urlopen(req,timeout=2): pass
    except OSError: pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path('deployment/bringup/jetson.json'))
    parser.add_argument('action',choices=['preflight','start','status','stop'])
    args=parser.parse_args()
    config=read_json(args.config.resolve(strict=True))
    name=config['name']
    if not name.replace('_','').replace('-','').isalnum(): parser.error('invalid profile name')
    state_dir=PROJECT/'.runtime/stack'/name;state_dir.mkdir(parents=True,exist_ok=True)
    with (state_dir/'lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        state_path=state_dir/'state.json';state=read_json(state_path)
        if args.action=='stop':
            if not state: print(json.dumps({'profile':name,'state':'not_started'}));return 0
            cancel_task(state)
            for item in reversed(state.get('components',[])): stop_owned(item)
            state['state']='stopped';atomic(state_path,state)
            print(json.dumps({'profile':name,'state':'stopped','external_services_preserved':True}));return 0
        if args.action=='status':
            output=[]
            for item in state.get('components',[]):
                healthy,detail=probe(item['spec'])
                output.append({'name':item['name'],'ownership':item['ownership'],'process_alive':owned_alive(item) if item['ownership']=='managed' else None,'healthy':healthy,'detail':detail})
            print(json.dumps({'profile':name,'state':state.get('state','not_started'),'run':state.get('run'),'components':output},indent=2));return 0
        run=PROJECT/'runs'/('imagegoal_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
        components=expand(config['components'],run)
        if args.action=='preflight':
            rows=[]
            for c in components:
                ok,detail=probe(c)
                rows.append({'name':c['name'],'already_healthy':ok,'detail':detail,'command':c['command']})
            print(json.dumps({'profile':name,'motion_transport':False,'components':rows},indent=2));return 0
        if any(owned_alive(v) for v in state.get('components',[])):
            parser.error('profile already has managed processes; use status or stop first')
        run.mkdir(parents=True,exist_ok=False);(run/'logs').mkdir()
        state={'schema_version':'imagegoal.stack.v1','name':name,'state':'starting','run':str(run),'runtime_url':config.get('runtime_url'),'components':[],'motion_transport':False}
        atomic(state_path,state);atomic(run/'resolved_profile.json',{'profile':config,'components':components})
        try:
            for c in components:
                healthy,detail=probe(c)
                item={'name':c['name'],'spec':c}
                if healthy:
                    if not c.get('adopt_existing',False): raise RuntimeError(c['name']+': endpoint already occupied; refusing to adopt')
                    item.update(ownership='external_healthy',detail=detail)
                else:
                    if unhealthy_http_port_occupied(c):
                        raise RuntimeError(c['name']+': endpoint occupied but health contract failed; preserving existing process: '+detail)
                    logfile=run/'logs'/(c['name']+'.log')
                    with logfile.open('ab') as stream:
                        p=subprocess.Popen(c['command'],cwd=PROJECT,env={**os.environ,**c.get('env',{})},stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                    item.update(ownership='managed',pid=p.pid,proc_start_token=proc_token(p.pid),log=str(logfile))
                    state['components'].append(item);atomic(state_path,state)
                    deadline=time.monotonic()+c.get('startup_timeout_s',30)
                    while time.monotonic()<deadline:
                        if p.poll() is not None: raise RuntimeError(f"{c['name']} exited {p.returncode}; inspect {logfile}")
                        healthy,detail=probe(c)
                        if healthy: break
                        time.sleep(.25)
                    if not healthy: raise RuntimeError(c['name']+': readiness failed: '+detail)
                    item['detail']=detail
                    if c['name']=='runtime':
                        with urlopen(c['health_url'],timeout=2) as stream: item['runtime_session_id']=json.load(stream)['session_id']
                if item not in state['components']: state['components'].append(item)
                atomic(state_path,state)
            state['state']='running';atomic(state_path,state)
            print(json.dumps({'profile':name,'state':'running','run':str(run),'runtime_url':state['runtime_url'],'motion_transport':False},indent=2));return 0
        except Exception as exc:
            cancel_task(state)
            for item in reversed(state['components']): stop_owned(item)
            state.update(state='failed',error=f'{type(exc).__name__}: {exc}');atomic(state_path,state)
            print(json.dumps(state,indent=2));return 1


if __name__=='__main__': raise SystemExit(main())
