"""Coordinate-frame and generated-artifact regressions."""

from __future__ import annotations

import json
import unittest

import numpy as np

from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.visualize import (
    END_EFFECTOR_FRAME_AXIS_LENGTH_M,
    TARGET_FRAME_AXIS_LENGTH_M,
    VIDEO_WAYPOINT_FRAME_AXIS_LENGTH_M,
    WAYPOINT_FRAME_AXIS_LENGTH_M,
    _frame_axis_endpoints,
)
from v6_mujoco.fpmfc.visualize_capture_multiview import (
    CAPTURE_VIEWS,
    _joint_kinematics,
)


class VisualizationFrameTests(unittest.TestCase):
    def test_five_capture_views_are_unique(self) -> None:
        self.assertEqual(len(CAPTURE_VIEWS), 5)
        self.assertEqual(len({view.key for view in CAPTURE_VIEWS}), 5)
        self.assertEqual(
            len({(view.azimuth_deg, view.elevation_deg) for view in CAPTURE_VIEWS}),
            5,
        )

    def test_joint_kinematics_uses_free_base_offsets_and_time_derivative(self) -> None:
        time = np.linspace(0.0, 1.0, 101)
        slopes = np.arange(1.0, 8.0)
        offsets = 0.1 * np.arange(7)
        trace = {
            "time": time,
            "qpos": np.zeros((len(time), 14)),
            "qvel": np.zeros((len(time), 13)),
        }
        trace["qpos"][:, 7:14] = offsets[None, :] + 0.5 * time[:, None] ** 2 * slopes[None, :]
        trace["qvel"][:, 6:13] = time[:, None] * slopes[None, :]
        q, dq, ddq = _joint_kinematics(trace)
        np.testing.assert_allclose(q, trace["qpos"][:, 7:14])
        np.testing.assert_allclose(dq, trace["qvel"][:, 6:13])
        np.testing.assert_allclose(ddq, np.broadcast_to(slopes, ddq.shape), atol=1.0e-12)

    def test_rgb_axes_follow_rotation_columns(self) -> None:
        origin = np.asarray([1.0, 2.0, 3.0])
        rotation = np.asarray([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        length = 0.2
        endpoints = _frame_axis_endpoints(origin, rotation, length)
        np.testing.assert_allclose(endpoints[:, 0], np.repeat(origin[None, :], 3, axis=0))
        np.testing.assert_allclose(endpoints[:, 1] - endpoints[:, 0], length * rotation.T)

    def test_v6_axis_lengths_are_preserved(self) -> None:
        self.assertEqual(TARGET_FRAME_AXIS_LENGTH_M, 0.160)
        self.assertEqual(END_EFFECTOR_FRAME_AXIS_LENGTH_M, 0.100)
        self.assertEqual(WAYPOINT_FRAME_AXIS_LENGTH_M, 0.050)
        self.assertEqual(VIDEO_WAYPOINT_FRAME_AXIS_LENGTH_M, 0.055)
        self.assertGreater(TARGET_FRAME_AXIS_LENGTH_M, END_EFFECTOR_FRAME_AXIS_LENGTH_M)

    def test_manifest_declares_both_synchronized_frames(self) -> None:
        path = PROJECT_ROOT / "output" / "visualization" / "visualization_manifest.json"
        manifest = json.loads(path.read_text(encoding="utf-8"))
        contract = manifest["coordinate_frames"]
        self.assertEqual(contract["axis_color_order"], ["x_red", "y_green", "z_blue"])
        self.assertCountEqual(contract["video_frames_rendered"], ["live_target_point_frame", "live_flexiv_flange_frame", "W1_to_W7_target_frames"])
        self.assertCountEqual(
            contract["path_figure_frames_rendered"],
            ["W1_to_W7_target_frames", "final_target_frame", "final_flexiv_flange_frame"],
        )
        self.assertEqual(manifest["video_target_path"]["waypoint_count"], 7)
        self.assertEqual(manifest["video_target_path"]["geometry"], "W1_to_W7_capsule_polyline")


if __name__ == "__main__":
    unittest.main()
