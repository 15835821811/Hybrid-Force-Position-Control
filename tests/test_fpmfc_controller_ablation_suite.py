"""Selection and contract checks for the formal N073 suite."""

from __future__ import annotations

import copy
import unittest

from v6_mujoco.fpmfc.controller_ablation_suite import (
    _all_finite,
    _select_first_qualified,
    _variant_config_contract,
)


class ControllerAblationSuiteTests(unittest.TestCase):
    def test_selection_uses_first_qualified_candidate_in_saved_order(self) -> None:
        candidates = [
            {"kinematic_rank": 1, "planning_and_dynamic_qualified": False},
            {"kinematic_rank": 2, "planning_and_dynamic_qualified": True},
            {"kinematic_rank": 3, "planning_and_dynamic_qualified": True},
        ]
        selected = _select_first_qualified(candidates)
        self.assertEqual(selected["kinematic_rank"], 2)

    def test_variant_contract_allows_only_declared_level2_weights(self) -> None:
        full = {
            "controller": {
                "shape_weight": 1.0,
                "base_reaction_weight": 2.0,
                "position_gain": 5.0,
            },
            "target": {"mode": "same"},
        }
        no_shape = copy.deepcopy(full)
        no_shape["controller"]["shape_weight"] = 0.0
        no_base = copy.deepcopy(full)
        no_base["controller"]["base_reaction_weight"] = 0.0
        configs = {
            "full": full,
            "no-shape": no_shape,
            "no-base-reaction": no_base,
        }
        self.assertTrue(_variant_config_contract(configs))
        configs["no-shape"]["target"]["mode"] = "changed"
        self.assertFalse(_variant_config_contract(configs))

    def test_finite_audit_rejects_nested_nonfinite_value(self) -> None:
        self.assertTrue(_all_finite({"a": [1.0, 2.0], "b": {"c": 3}}))
        self.assertFalse(_all_finite({"a": [1.0, float("nan")]}))


if __name__ == "__main__":
    unittest.main()
