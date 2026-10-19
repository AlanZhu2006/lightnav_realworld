"""Exercise one-click model preparation without SSH, GPU or task mutations."""
from contextlib import redirect_stdout
import io
import hashlib
import json
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from deployment.operator import open_console as console


def health():
    value = {}
    for key, expected in console.HEALTH_EXPECT.items():
        node = value
        parts = key.split('.')
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = expected
    return value


def response(value):
    return io.BytesIO(json.dumps(value).encode())


class LaunchTests(unittest.TestCase):
    def report(self, **updates):
        return {'ok': True, 'session_id': 'active-session', 'state': 'PLANNING',
                'hardware_transport': 'absent', 'odom_available': True,
                'nomad_ready': True, 'lightnav_ready': True,
                'lightnav_endpoint_present': True, **updates}

    def launch(self, report, argv, post_forward=True):
        replies = [subprocess.CompletedProcess([], 0, json.dumps(report), '')]
        replies.append(subprocess.CompletedProcess([], 0 if post_forward else 1,
                                                   'ready\n' if post_forward else '', 'bad contract'))
        with patch.object(console.subprocess, 'run', side_effect=replies) as run, \
             patch.object(console, 'tunnel', return_value=18766) as tunnel, \
             patch.object(console, 'ensure_local_lightnav', return_value={'action': 'already_healthy'}) as prepare, \
             patch.object(console, 'urlopen', side_effect=lambda *a, **k: response(health())) as get, \
             patch.object(console.webbrowser, 'open') as browser, \
             redirect_stdout(io.StringIO()):
            result = console.main(argv)
        return result, run, tunnel, prepare, get, browser

    def test_existing_remote_model_preserves_active_task_and_avoids_local_gpu(self):
        result, run, tunnel, prepare, get, browser = self.launch(
            self.report(), ['--backend', 'lightnav', '--no-open'])
        self.assertEqual(result, 0)
        prepare.assert_not_called()
        get.assert_not_called()
        browser.assert_not_called()
        self.assertEqual(run.call_count, 1)
        tunnel.assert_called_once_with('go2', 18766, 0, 18884)
        self.assertNotIn('/tasks', run.call_args.kwargs['input'])
        self.assertNotIn('/reset', run.call_args.kwargs['input'])

    def test_missing_remote_prepares_model_then_validates_reverse_connection(self):
        result, run, tunnel, prepare, get, _ = self.launch(
            self.report(lightnav_ready=False, lightnav_endpoint_present=False),
            ['--backend', 'lightnav', '--no-open'])
        self.assertEqual(result, 0)
        prepare.assert_called_once_with(project=console.PROJECT, port=18884)
        get.assert_called_once_with('http://127.0.0.1:18884/healthz', timeout=3)
        tunnel.assert_called_once_with('go2', 18766, 18884, 18884)
        self.assertEqual(run.call_count, 2)
        self.assertIn('contract.model_revision', run.call_args.kwargs['input'])
        self.assertIn('contract.model_sha256', run.call_args.kwargs['input'])

    def test_occupied_remote_endpoint_is_preserved_without_gpu_or_tunnel_changes(self):
        result, _, tunnel, prepare, get, _ = self.launch(
            self.report(lightnav_ready=False), ['--backend', 'lightnav', '--no-open'])
        self.assertEqual(result, 1)
        prepare.assert_not_called()
        tunnel.assert_not_called()
        get.assert_not_called()

    def test_legacy_port_flag_does_not_implicitly_launch_a_gpu_model(self):
        result, _, _, prepare, _, _ = self.launch(
            self.report(lightnav_ready=False, lightnav_endpoint_present=False),
            ['--lightnav-local-port', '18884', '--no-open'])
        self.assertEqual(result, 0)
        prepare.assert_not_called()

    def test_failed_post_forward_health_is_not_reported_ready(self):
        result, _, _, _, _, browser = self.launch(
            self.report(lightnav_ready=False, lightnav_endpoint_present=False),
            ['--backend', 'lightnav'], post_forward=False)
        self.assertEqual(result, 1)
        browser.assert_not_called()

    def test_missing_nomad_preserves_runtime_instead_of_restarting_it(self):
        result, run, tunnel, prepare, _, _ = self.launch(
            self.report(nomad_ready=False), ['--backend', 'nomad', '--no-open'])
        self.assertEqual(result, 1)
        self.assertEqual(run.call_count, 1)
        prepare.assert_not_called()
        tunnel.assert_not_called()


class RemoteEnsureTests(unittest.TestCase):
    def runtime(self):
        return {'schema_version': 'imagegoal.runtime.v1', 'session_id': 'keep-this-session',
                'state': 'PLANNING', 'hardware_transport': 'absent',
                'task': {'task_id': 'keep-this-task'}, 'sensors': {'odom': {}}}

    def execute(self, runtime_first=True):
        runtime_calls = []

        def get(url, **kwargs):
            self.assertIsInstance(url, str)  # read-only GET, never a task POST
            if ':8093/status' in url:
                runtime_calls.append(url)
                if not runtime_first and len(runtime_calls) == 1:
                    raise OSError('offline')
                return response(self.runtime())
            if ':18884/healthz' in url:
                return response(health())
            if ':18883/healthz' in url:
                return response({'ok': True, 'model_loaded': True})
            self.fail(url)

        with patch('urllib.request.urlopen', side_effect=get), \
             patch('socket.create_connection'), \
             patch('subprocess.run', return_value=subprocess.CompletedProcess([], 0, '{}', '')) as run, \
             redirect_stdout(io.StringIO()) as output:
            exec(console.REMOTE_ENSURE, {})
        return json.loads(output.getvalue()), run

    def test_healthy_runtime_task_and_session_are_left_intact(self):
        report, run = self.execute()
        self.assertEqual(report['session_id'], 'keep-this-session')
        self.assertEqual(report['state'], 'PLANNING')
        self.assertTrue(report['lightnav_ready'])
        run.assert_not_called()

    def test_cold_start_uses_current_observation_helper(self):
        report, run = self.execute(runtime_first=False)
        self.assertEqual(report['session_id'], 'keep-this-session')
        self.assertEqual(run.call_args.args[0][1:], ['-m', 'deployment.bringup.ensure_observation'])
        self.assertNotIn('jetson.json', console.REMOTE_ENSURE)

    def test_post_forward_check_rejects_wrong_model_revision(self):
        wrong = health()
        wrong['contract']['model_revision'] = 'different'
        with patch('urllib.request.urlopen', return_value=response(wrong)), \
             self.assertRaises(AssertionError):
            exec(console.remote_lightnav_check(), {})


class NativePreparationTests(unittest.TestCase):
    def execute(self, state='IDLE', already=False, bad_image=False, change_epoch=False, lightnav_only=False):
        image=b'\xff\xd8original-goal\xff\xd9'
        digest=hashlib.sha256(image).hexdigest()
        live={'session_id':'old','epoch':3,'state':state,'task':None,
              'execution_capability':{'supervised_native_configured':already,'operator_supervised':already},
              'goal_selection':{'ready':True,'selection_id':'selected','sha256':digest}}
        started=[]
        reads=[]

        def get(url, **kwargs):
            self.assertIsInstance(url,str)  # preparation has no task POST
            if '/goal.jpg?' in url:
                return io.BytesIO(b'wrong' if bad_image else image)
            reads.append(url)
            if started:
                return response({'session_id':'new','task':None,'native_executor':{'running':False},
                                 'execution_capability':{'supervised_native_configured':True},
                                 'goal_selection':{'sha256':digest}})
            return response({**live,'epoch':4} if change_epoch and len(reads)>1 else live)

        def run(command, **kwargs):
            started.append(command)
            return subprocess.CompletedProcess(command,0,'prepared','')

        with tempfile.TemporaryDirectory() as directory, \
             patch('urllib.request.urlopen',side_effect=get), \
             patch('subprocess.run',side_effect=run) as process, \
             redirect_stdout(io.StringIO()) as output:
            code=console.REMOTE_PREPARE_NATIVE.replace("'/home/unitree/work/lightnav_realworld'",repr(directory))
            if lightnav_only:
                code=code.replace('lightnav_only = False','lightnav_only = True')
            error=None
            try:
                exec(code,{})
            except SystemExit as exc:
                error=str(exc)
        return output.getvalue(),error,process,started

    def test_prepared_runtime_is_adopted_even_with_an_active_task(self):
        out,error,process,_=self.execute(state='TRACKING',already=True)
        self.assertIsNone(error)
        self.assertEqual(json.loads(out)['action'],'already_prepared')
        process.assert_not_called()

    def test_active_shadow_task_is_not_restarted(self):
        _,error,process,_=self.execute(state='PLANNING')
        self.assertIn('当前有任务运行',error)
        process.assert_not_called()

    def test_prepare_only_preserves_exact_goal_and_never_executes_motion(self):
        out,error,process,started=self.execute()
        self.assertIsNone(error)
        self.assertEqual(json.loads(out)['action'],'prepared_no_task_or_executor')
        process.assert_called_once()
        command=started[0]
        self.assertIn('--prepare-only',command)
        self.assertIn('--goal',command)
        self.assertNotIn('--execute-supervised',command)

    def test_lightnav_preparation_selects_no_nomad_model_profile(self):
        _,error,_,started=self.execute(lightnav_only=True)
        self.assertIsNone(error)
        command=started[0]
        self.assertEqual(command[command.index('--stack-profile')+1],'lightnav')
        self.assertIn('--prepare-only',command)
        self.assertNotIn('--execute-supervised',command)

    def test_corrupt_goal_does_not_change_services(self):
        _,error,process,_=self.execute(bad_image=True)
        self.assertIn('目标图不完整',error)
        process.assert_not_called()

    def test_changed_epoch_does_not_replace_a_newer_task_or_goal(self):
        _,error,process,_=self.execute(change_epoch=True)
        self.assertIn('发生变化',error)
        process.assert_not_called()


if __name__ == '__main__':
    unittest.main()
