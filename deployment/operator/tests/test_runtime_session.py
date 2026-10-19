"""Mutation guards must also reject commands crossing a runtime restart."""
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import patch
from deployment.imagegoal.runtime import ImageGoalRuntime

class RuntimeSessionTests(unittest.TestCase):
    def setUp(self):
        self.runtime=ImageGoalRuntime.__new__(ImageGoalRuntime)
        self.runtime.session_id="new-session"
        self.runtime.epoch=0
        self.runtime.lock=threading.RLock()
        self.runtime.config=SimpleNamespace(execution_profile="shadow")
        self.runtime.task={"execution_mode":"shadow"}
        self.runtime.state="PAUSED"

    def test_restart_with_same_epoch_rejects_old_start(self):
        with self.assertRaisesRegex(ValueError,"expected_session_id mismatch"):
            self.runtime.create_task({"expected_session_id":"old-session","expected_epoch":0})
        self.assertEqual(self.runtime.epoch,0)

    def test_old_session_cannot_mutate_pause_resume_cancel_or_complete(self):
        for action in ("pause","resume","cancel","complete"):
            with self.subTest(action=action), self.assertRaisesRegex(ValueError,"expected_session_id mismatch"):
                self.runtime.task_action(action,{"expected_session_id":"old-session"})
        self.assertEqual(self.runtime.state,"PAUSED")
        self.assertEqual(self.runtime.epoch,0)

    def test_old_session_cannot_replace_goal(self):
        with patch("deployment.imagegoal.runtime.decode_jpeg",return_value=b"jpeg"), self.assertRaisesRegex(ValueError,"expected_session_id mismatch"):
            self.runtime.replace_goal({"expected_session_id":"old-session","expected_epoch":0})
        self.assertEqual(self.runtime.epoch,0)

    def test_stale_task_actions_cannot_affect_a_new_task(self):
        for action in ("pause","resume","complete"):
            with self.subTest(action=action), self.assertRaisesRegex(ValueError,"expected_epoch mismatch"):
                self.runtime.task_action(action,{"expected_session_id":"new-session","expected_epoch":99})
        self.assertEqual(self.runtime.epoch,0)

    def test_matching_session_and_legacy_epoch_checks_remain_valid(self):
        self.runtime._check_expected_epoch({"expected_session_id":"new-session","expected_epoch":0})
        self.runtime._check_expected_epoch({"expected_epoch":0})
        self.runtime._check_expected_epoch({})
        with self.assertRaisesRegex(ValueError,"expected_epoch mismatch"):
            self.runtime._check_expected_epoch({"expected_session_id":"new-session","expected_epoch":1})

if __name__=="__main__": unittest.main()
