"""Connection cache must add newly requested LightNav forwarding."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

path=Path(__file__).resolve().parents[1]/'open_console.py'
spec=importlib.util.spec_from_file_location('imagegoal_launcher',path)
launcher=importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)

class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.host='unit-test-'+uuid.uuid4().hex
        self.preferred=48766
        self.variant=18884
        digest=hashlib.sha256(f'{self.host}:{self.preferred}:{self.variant}'.encode()).hexdigest()[:12]
        self.directory=Path('/tmp')/f'dvfg-{os.getuid()}-{digest}'
        self.directory.mkdir(mode=0o700)
        (self.directory/'connection.json').write_text(json.dumps({'port':self.preferred,'host':self.host}))

    def tearDown(self):
        shutil.rmtree(self.directory)

    def test_cached_master_adds_missing_reverse_forward(self):
        calls=[]
        def run(command,**kwargs):
            calls.append(command)
            return SimpleNamespace(returncode=0,stdout='',stderr='')
        with patch.object(launcher.subprocess,'run',side_effect=run),patch.object(launcher,'websocket_ready',return_value=True):
            self.assertEqual(launcher.tunnel(self.host,self.preferred,self.variant,self.variant),self.preferred)
        forwards=[c for c in calls if 'forward' in c]
        self.assertEqual(len(forwards),1)
        self.assertIn('127.0.0.1:18884:127.0.0.1:18884',forwards[0])
        self.assertEqual(json.loads((self.directory/'connection.json').read_text())['reverse_lightnav_port'],self.variant)

    def test_reverse_forward_failure_is_reported_and_not_cached(self):
        def run(command,**kwargs):
            return SimpleNamespace(returncode=1 if 'forward' in command else 0,stdout='',stderr='forward failed')
        with patch.object(launcher.subprocess,'run',side_effect=run),patch.object(launcher,'websocket_ready',return_value=True):
            with self.assertRaisesRegex(RuntimeError,'forward failed'):
                launcher.tunnel(self.host,self.preferred,self.variant,self.variant)
        self.assertNotIn('reverse_lightnav_port',json.loads((self.directory/'connection.json').read_text()))

if __name__=='__main__': unittest.main()
