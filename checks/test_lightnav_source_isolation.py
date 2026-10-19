"""CPU provenance checks; no model, CUDA, ROS, camera, or robot is initialized."""

from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

from deployment.lightnav import policy


ROOT = Path(__file__).resolve().parents[1]


class SourceIsolationTests(unittest.TestCase):
    def test_unrelated_modules_do_not_require_lightnav_origins(self):
        self.assertEqual(policy.verify_loaded_upstream_sources({"unrelated": object()}), 0)

    def test_loaded_pinned_packages_and_modules_are_accepted(self):
        package = policy.VENDOR / "src/lightnav"
        modules = {
            "lightnav": SimpleNamespace(
                __file__=str(package / "__init__.py"), __path__=[str(package)]
            ),
            "lightnav.prompts": SimpleNamespace(__file__=str(package / "prompts.py")),
        }
        self.assertEqual(policy.verify_loaded_upstream_sources(modules), 2)

    def test_repository_root_package_is_rejected_even_if_bytes_match(self):
        package = ROOT / "src/lightnav"
        with self.assertRaisesRegex(RuntimeError, "separate Python process"):
            policy.verify_loaded_upstream_sources(
                {"lightnav": SimpleNamespace(__file__=str(package / "__init__.py"))}
            )

    def test_mixed_submodule_and_extended_package_paths_are_rejected(self):
        package = policy.VENDOR / "src/lightnav"
        for modules in (
            {
                "lightnav": SimpleNamespace(__file__=str(package / "__init__.py")),
                "lightnav.prompts": SimpleNamespace(__file__=str(ROOT / "src/lightnav/prompts.py")),
            },
            {
                "lightnav": SimpleNamespace(
                    __file__=str(package / "__init__.py"),
                    __path__=[str(package), str(ROOT / "src/lightnav")],
                )
            },
        ):
            with self.subTest(modules=tuple(modules)):
                with self.assertRaisesRegex(RuntimeError, "source isolation failed"):
                    policy.verify_loaded_upstream_sources(modules)

    def test_unverified_and_missing_origins_are_rejected(self):
        for module in (
            None,
            SimpleNamespace(),
            SimpleNamespace(__file__=str(policy.VENDOR / "src/lightnav/serving/ws_server.py")),
        ):
            with self.subTest(module=module):
                with self.assertRaisesRegex(RuntimeError, "source isolation failed"):
                    policy.verify_loaded_upstream_sources({"lightnav.unknown": module})

    def run_fresh_python(self, body):
        result = subprocess.run(
            [sys.executable, "-B", "-c", body],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=15,
            check=True,
        )
        return json.loads(result.stdout)

    def test_fresh_process_imports_the_pinned_core_without_torch(self):
        result = self.run_fresh_python(
            "import json, sys\n"
            "from deployment.lightnav.policy import activate_verified_upstream, verify_loaded_upstream_sources\n"
            "activate_verified_upstream()\n"
            "import lightnav, lightnav.prompts\n"
            "print(json.dumps({'count': verify_loaded_upstream_sources(), "
            "'torch_imported': 'torch' in sys.modules, 'origin': lightnav.__file__}))\n"
        )
        self.assertEqual(result["count"], 2)
        self.assertFalse(result["torch_imported"])
        self.assertTrue(Path(result["origin"]).is_relative_to(policy.VENDOR.resolve()))

    def test_preloaded_original_package_fails_without_replacing_it(self):
        result = self.run_fresh_python(
            "import json, pathlib, sys\n"
            "sys.path.insert(0, str(pathlib.Path('src').resolve()))\n"
            "import lightnav\n"
            "original = lightnav\n"
            "from deployment.lightnav.policy import activate_verified_upstream\n"
            "try:\n"
            " activate_verified_upstream()\n"
            "except RuntimeError as error:\n"
            " print(json.dumps({'rejected': True, 'preserved': sys.modules['lightnav'] is original, "
            "'torch_imported': 'torch' in sys.modules, 'error': str(error)}))\n"
            "else:\n"
            " raise SystemExit('preloaded original package was incorrectly accepted')\n"
        )
        self.assertTrue(result["rejected"])
        self.assertTrue(result["preserved"])
        self.assertFalse(result["torch_imported"])


if __name__ == "__main__":
    unittest.main()
