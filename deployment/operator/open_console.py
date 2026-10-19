#!/usr/bin/env python3
"""Connect the ImageGoal Foxglove console using one SSH tunnel (no motion commands)."""
from __future__ import annotations
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
from urllib.parse import urlencode
import webbrowser
from urllib.request import urlopen

PROJECT = Path(__file__).resolve().parents[2]
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))
from deployment.lightnav.launcher import HEALTH_EXPECT, ensure_local_lightnav, health_matches

REMOTE_ENSURE = r'''
import json, pathlib, socket, subprocess, sys, urllib.request
root=pathlib.Path('/home/unitree/work/lightnav_realworld')
lightnav_only = False
def status():
    try:
        with urllib.request.urlopen('http://127.0.0.1:8093/status', timeout=3) as response:
            value=json.load(response)
        return value if value.get('schema_version')=='imagegoal.runtime.v1' else None
    except Exception:
        return None
value=status()
if value is None:
    command=[sys.executable,'-m','deployment.bringup.ensure_observation']
    if lightnav_only:
        command+=['--config','deployment/bringup/jetson_lightnav.json']
    result=subprocess.run(command,
                          cwd=root,capture_output=True,text=True,timeout=260)
    if result.returncode:
        raise SystemExit('观测服务未就绪；保留现有进程。'+(result.stdout+result.stderr)[-2000:])
    value=status()
if value is None:
    raise SystemExit('ImageGoal 服务未就绪。')
with socket.create_connection(('127.0.0.1',8766),timeout=3):
    pass
def model_ready(port):
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{port}/healthz',timeout=2) as response:
            health=json.load(response)
        return health.get('ok') is True and health.get('model_loaded') is True
    except Exception:
        return False
def matches(health):
    for key,expected in __LIGHTNAV_HEALTH_EXPECT__.items():
        actual=health
        for part in key.split('.'):
            actual=actual.get(part) if isinstance(actual,dict) else None
        if type(actual) is not type(expected) or actual!=expected:
            return False
    return True
try:
    with urllib.request.urlopen('http://127.0.0.1:18884/healthz',timeout=3) as response:
        lightnav=matches(json.load(response))
except Exception:
    lightnav=False
try:
    with socket.create_connection(('127.0.0.1',18884),timeout=1):
        lightnav_endpoint_present=True
except OSError:
    lightnav_endpoint_present=False
print(json.dumps({'ok':True,'session_id':value['session_id'],'state':value['state'],'hardware_transport':value.get('hardware_transport'),'odom_available':value.get('sensors',{}).get('odom') is not None,'nomad_ready':model_ready(18883),'lightnav_ready':lightnav,'lightnav_endpoint_present':lightnav_endpoint_present,'native_configured':value.get('execution_capability',{}).get('supervised_native_configured') is True}))
'''
REMOTE_ENSURE = REMOTE_ENSURE.replace('__LIGHTNAV_HEALTH_EXPECT__', repr(HEALTH_EXPECT))

REMOTE_PREPARE_NATIVE = r'''
import hashlib,json,pathlib,subprocess,urllib.request
root=pathlib.Path('/home/unitree/work/lightnav_realworld')
lightnav_only = False
def status():
    try:
        with urllib.request.urlopen('http://127.0.0.1:8093/status',timeout=3) as response:
            return json.load(response)
    except OSError:
        return None
value=status()
capability=(value or {}).get('execution_capability') or {}
if capability.get('supervised_native_configured') is True and capability.get('operator_supervised') is True:
    print(json.dumps({'action':'already_prepared','session_id':value['session_id']}))
else:
    if value is not None and (value.get('state') not in {'IDLE','CANCELLED','SUCCEEDED','FAILED','TIMED_OUT'}
                              or (value.get('native_executor') or {}).get('running') is True):
        raise SystemExit('当前有任务运行；先用面板停止任务，再准备真机按钮。现有任务保持原状态。')
    command=['bash','deployment/go2/run_go2.sh','native-imagegoal','--prepare-only','--operator-present']
    if lightnav_only:
        command+=['--stack-profile','lightnav']
    selection=(value or {}).get('goal_selection') or {}
    if selection.get('ready') is True:
        with urllib.request.urlopen('http://127.0.0.1:8093/goal.jpg?selection_id='+selection['selection_id'],timeout=4) as response:
            image=response.read(4*1024*1024+1)
        digest=hashlib.sha256(image).hexdigest()
        if len(image)>4*1024*1024 or digest!=selection.get('sha256'):
            raise SystemExit('当前目标图不完整，保持原观测服务。')
        directory=root/'.runtime/operator_native_prepare'
        directory.mkdir(parents=True,exist_ok=True)
        path=directory/(digest+'.jpg');path.write_bytes(image)
        command+=['--goal',str(path)]
    latest=status()
    if value is not None and (latest is None or latest.get('session_id')!=value.get('session_id')
            or latest.get('epoch')!=value.get('epoch') or latest.get('state')!=value.get('state')):
        raise SystemExit('当前任务或目标发生变化，保持原服务；请稍后再准备真机按钮。')
    # Preparation configures the observation service. The native executor is
    # launched only by a later explicit Foxglove native_start command.
    result=subprocess.run(command,cwd=root,capture_output=True,text=True,timeout=260)
    if result.returncode:
        raise SystemExit('真机按钮准备失败：'+(result.stdout+result.stderr)[-3000:])
    fresh=status()
    if (fresh is None or (fresh.get('execution_capability') or {}).get('supervised_native_configured') is not True
            or (fresh.get('native_executor') or {}).get('running') is True or fresh.get('task') is not None):
        raise SystemExit('观测配置未达到真机按钮待启动状态。')
    if selection.get('ready') is True and (fresh.get('goal_selection') or {}).get('sha256')!=selection.get('sha256'):
        raise SystemExit('原目标图未恢复，请检查准备日志。')
    print(json.dumps({'action':'prepared_no_task_or_executor','session_id':fresh['session_id']}))
'''


def remote_lightnav_check() -> str:
    return ("import json,urllib.request\n"
            "h=json.load(urllib.request.urlopen('http://127.0.0.1:18884/healthz',timeout=4))\n"
            f"expected={HEALTH_EXPECT!r}\n"
            "for key,value in expected.items():\n"
            " actual=h\n"
            " for part in key.split('.'):\n"
            "  actual=actual.get(part) if isinstance(actual,dict) else None\n"
            " assert type(actual) is type(value) and actual==value, f'{key}: {actual!r}'\n"
            "print('ready')\n")


def websocket_ready(port: int) -> bool:
    """Require a real Foxglove protocol upgrade, not just an open TCP socket."""
    key=base64.b64encode(os.urandom(16)).decode()
    expected=base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
    request=(f'GET / HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: foxglove.sdk.v1\r\n\r\n')
    try:
        with socket.create_connection(('127.0.0.1',port),timeout=3) as stream:
            stream.sendall(request.encode())
            response=b''
            while b'\r\n\r\n' not in response and len(response)<16384:
                chunk=stream.recv(2048)
                if not chunk: return False
                response+=chunk
        head=response.split(b'\r\n\r\n',1)[0].decode('latin1')
        lines=head.split('\r\n')
        headers={key.strip().lower():value.strip() for line in lines[1:] if ':' in line for key,value in [line.split(':',1)]}
        return ' 101 ' in lines[0] and headers.get('sec-websocket-accept')==expected and headers.get('sec-websocket-protocol')=='foxglove.sdk.v1'
    except OSError:
        return False


def tunnel(host: str, preferred: int, reverse_lightnav_port: int = 0, cache_variant: int = 0) -> int:
    uid=os.getuid() if hasattr(os,'getuid') else 0
    cache_key=f'{host}:{preferred}:{cache_variant}' if cache_variant else f'{host}:{preferred}'
    token=hashlib.sha256(cache_key.encode()).hexdigest()[:12]
    directory=Path('/tmp')/f'dvfg-{uid}-{token}'
    directory.mkdir(mode=0o700,exist_ok=True)
    if directory.is_symlink() or (hasattr(os,'getuid') and directory.stat().st_uid!=uid):
        raise RuntimeError('连接缓存目录的所有者不匹配。')
    directory.chmod(0o700)
    import fcntl
    with (directory/'lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        control=str(directory/'ssh')
        state_path=directory/'connection.json'
        check=subprocess.run(['ssh','-S',control,'-O','check',host],capture_output=True)
        if check.returncode==0:
            if not state_path.exists():
                raise RuntimeError('已有连接尚未写入端口信息，请稍后重试。')
            state=json.loads(state_path.read_text())
            if websocket_ready(state['port']):
                if reverse_lightnav_port and state.get('reverse_lightnav_port')!=reverse_lightnav_port:
                    result=subprocess.run(['ssh','-S',control,'-O','forward','-o','ExitOnForwardFailure=yes','-R',f'127.0.0.1:18884:127.0.0.1:{reverse_lightnav_port}',host],capture_output=True,text=True)
                    if result.returncode: raise RuntimeError(result.stderr.strip() or 'LightNav 反向连接失败。')
                    state['reverse_lightnav_port']=reverse_lightnav_port
                    state_path.write_text(json.dumps(state)+'\n')
                return state['port']
            raise RuntimeError('已有隧道的 Foxglove 服务没有响应；远端服务可能正在重启，请稍后重试。')
        for port in range(preferred,min(65536,preferred+20)):
            try:
                with socket.socket() as test: test.bind(('127.0.0.1',port))
            except OSError:
                continue
            forward=['-R',f'127.0.0.1:18884:127.0.0.1:{reverse_lightnav_port}'] if reverse_lightnav_port else []
            result=subprocess.run(['ssh','-M','-S',control,'-fNT','-o','BatchMode=yes','-o','ConnectTimeout=12','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=20','-o','ServerAliveCountMax=3','-L',f'127.0.0.1:{port}:127.0.0.1:8766',*forward,host],capture_output=True,text=True)
            if result.returncode:
                raise RuntimeError(result.stderr.strip() or 'SSH 隧道启动失败。')
            if not websocket_ready(port):
                subprocess.run(['ssh','-S',control,'-O','exit',host],capture_output=True)
                raise RuntimeError('SSH 已连接，但没有收到 Foxglove WebSocket 协议响应。')
            state_path.write_text(json.dumps({'port':port,'host':host,'reverse_lightnav_port':reverse_lightnav_port})+'\n')
            return port
        raise RuntimeError('本地可用端口不足。')


def main(argv=None) -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default=os.environ.get('LIGHTNAV_ROBOT_SSH_HOST','go2'))
    parser.add_argument('--remote-root',default=os.environ.get('LIGHTNAV_ROBOT_PROJECT','/home/unitree/work/lightnav_realworld'),
                        help='absolute checkout on the robot; never implicitly target another project')
    parser.add_argument('--port',type=int,default=18766)
    parser.add_argument('--no-open',action='store_true')
    parser.add_argument('--layout-id',help='optional saved Foxglove layout ID after first import')
    parser.add_argument('--backend',choices=('auto','nomad','lightnav'),default='auto',
                        help='prepare the selected model; tasks are still started explicitly in Foxglove')
    parser.add_argument('--prepare-native',action='store_true',
                        help='prepare the existing native observation profile; motion starts only from the panel button')
    parser.add_argument('--stack-profile',choices=('combined','lightnav'),
                        help='defaults to lightnav for --backend lightnav; combined also starts optional NoMaD')
    parser.add_argument('--lightnav-local-port',type=int,default=0,help='optional workstation LightNav HTTP port; forwarded only if Jetson LightNav is offline')
    args=parser.parse_args(argv)
    if not args.remote_root.startswith('/') or '\x00' in args.remote_root or '\n' in args.remote_root:
        parser.error('remote-root must be an absolute path without control characters')
    remote_ensure = REMOTE_ENSURE.replace(repr('/home/unitree/work/lightnav_realworld'),repr(args.remote_root))
    remote_prepare = REMOTE_PREPARE_NATIVE.replace(repr('/home/unitree/work/lightnav_realworld'),repr(args.remote_root))
    if (args.stack_profile or ('lightnav' if args.backend=='lightnav' else 'combined'))=='lightnav':
        remote_ensure = remote_ensure.replace('lightnav_only = False','lightnav_only = True')
        remote_prepare = remote_prepare.replace('lightnav_only = False','lightnav_only = True')
    if args.lightnav_local_port and not 1024<=args.lightnav_local_port<=65535: parser.error('LightNav port must be 1024..65535')
    if not 1024<=args.port<=65515: parser.error('port must be 1024..65515')
    if args.host.startswith('-') or not args.host.replace('-','').replace('_','').replace('.','').isalnum(): parser.error('host must be an SSH alias')
    try:
        if args.prepare_native:
            print('正在准备真机按钮；任务由面板点击启动…',flush=True)
            prepared=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=12',args.host,'python3','-'],
                                    input=remote_prepare,text=True,capture_output=True,timeout=300)
            if prepared.returncode:
                raise RuntimeError(prepared.stderr.strip() or prepared.stdout.strip() or '真机按钮准备失败。')
        result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=12',args.host,'python3','-'],input=remote_ensure,text=True,capture_output=True,timeout=300)
        if result.returncode: raise RuntimeError(result.stderr.strip() or result.stdout.strip() or '远端服务检查失败。')
        report=json.loads(result.stdout.strip().splitlines()[-1])
        if args.prepare_native and report.get('native_configured') is not True:
            raise RuntimeError('运行时已变化，真机按钮未准备；保留当前服务。')
        reverse_port=0
        local_port=args.lightnav_local_port or (18884 if args.backend=='lightnav' else 0)
        if args.backend=='nomad' and not report.get('nomad_ready'):
            raise RuntimeError('下位机 NoMaD 尚未就绪；当前任务服务已保留，请检查模型日志。')
        if local_port and not report.get('lightnav_ready'):
            if report.get('lightnav_endpoint_present'):
                raise RuntimeError('下位机 LightNav 端口已被占用，但模型合同不符；保留连接，请检查现有服务。')
            if args.backend=='lightnav':
                print('正在准备 LightNav 模型服务…',flush=True)
                prepared=ensure_local_lightnav(project=PROJECT,port=local_port)
                print('LightNav 模型服务已就绪：'+prepared['action'],flush=True)
            with urlopen(f'http://127.0.0.1:{local_port}/healthz',timeout=3) as response:
                health=json.load(response)
            if not health_matches(health):
                raise RuntimeError('本地 LightNav 模型名称、版本或权重合同不符；保留现有服务。')
            reverse_port=local_port
        port=tunnel(args.host,args.port,reverse_port,local_port)
        if reverse_port:
            check=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10',args.host,'python3','-'],input=remote_lightnav_check(),text=True,capture_output=True,timeout=15)
            if check.returncode or check.stdout.strip()!='ready':
                raise RuntimeError('LightNav 反向转发已建立，但远端模型健康检查未通过。')
            report['lightnav_ready']=True
        query={'ds':'foxglove-websocket','ds.url':f'ws://127.0.0.1:{port}','openIn':'web'}
        if args.layout_id: query['layoutId']=args.layout_id
        url='https://app.foxglove.dev/~/view?'+urlencode(query)
        print('ImageGoal 已连接；任务状态：'+report['state'])
        print('真实 odom：'+('可用' if report['odom_available'] else '缺失；模型局部路径仍可显示'))
        print('NoMaD：'+('在线' if report.get('nomad_ready') else '离线')+'；LightNav：'+('在线' if report.get('lightnav_ready') else '离线（其他界面功能仍可用）'))
        if args.backend!='auto':
            model='LightNav' if args.backend=='lightnav' else 'NoMaD'
            print('模型已准备；在面板选择 '+model+'，采集或选择目标后，点击“启动 '+model+' 真机”行走，或点击普通开始运行影子任务。'
                  if report.get('native_configured') is True else
                  '模型已准备；在面板选择 '+model+' 后可开始影子任务；追加 --prepare-native 可准备真机启动按钮。')
        print(url)
        if not args.no_open: webbrowser.open(url)
        return 0
    except (OSError,RuntimeError,ValueError,subprocess.TimeoutExpired) as error:
        print('连接失败：'+str(error),file=sys.stderr)
        return 1


if __name__=='__main__':
    raise SystemExit(main())
