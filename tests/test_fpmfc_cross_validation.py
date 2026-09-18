"""Regression gate for the exported Simscape/MuJoCo Jacobian cross-check."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from v6_mujoco.fpmfc.cross_validate_simscape import cross_validate
from v6_mujoco.model import PROJECT_ROOT


class TestFPMFCCrossValidation(unittest.TestCase):
    def test_home_jacobian_export_remains_within_audited_tolerances(self) -> None:
        source = PROJECT_ROOT / "diagnostics" / "simscape_jacobian_home.mat"
        self.assertTrue(source.is_file())
        with tempfile.TemporaryDirectory() as temporary:
            report = cross_validate(source, Path(temporary) / "report.json")
        self.assertTrue(report["passed"])
        self.assertLessEqual(report["metrics"]["flange_position_error_m"], 1e-3)
        self.assertLessEqual(report["metrics"]["jg_relative_frobenius_error"], 5e-3)
        self.assertLessEqual(
            report["metrics"]["reaction_relative_frobenius_error"], 3e-2
        )


if __name__ == "__main__":
    unittest.main()
