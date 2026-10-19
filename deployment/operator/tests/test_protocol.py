import json
import unittest

from deployment.imagegoal.foxglove_control import parse_command


class ProtocolTests(unittest.TestCase):
    def raw(self, action="start", payload=None, **kwargs):
        value = {"request_id": "request-1", "session_id": "session-1", "action": action,
                 "payload": {"expected_epoch": 0} if payload is None else payload}
        value.update(kwargs)
        return json.dumps(value)

    def test_start_and_goal_require_explicit_integer_epoch(self):
        for action in ("start", "replace_goal", "capture_goal", "import_goal", "clear_goal"):
            for payload in ({}, {"expected_epoch": True}, {"expected_epoch": -1}):
                with self.assertRaises(ValueError):
                    parse_command(self.raw(action=action, payload=payload))

    def test_reject_unlisted_actions_and_invalid_identifiers(self):
        for raw in (self.raw(action="/inputs/odom"), self.raw(request_id="../bad"), self.raw(session_id="")):
            with self.assertRaises(ValueError):
                parse_command(raw)

    def test_reject_nonfinite_numbers(self):
        for token in ("NaN", "Infinity", "1e999"):
            raw = self.raw(payload={}).replace('"payload": {}', '"payload": {"x": ' + token + '}')
            with self.assertRaises(ValueError):
                parse_command(raw)


if __name__ == "__main__":
    unittest.main()
