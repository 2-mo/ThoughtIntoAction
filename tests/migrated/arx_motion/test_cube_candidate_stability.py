from __future__ import annotations

import unittest

from arx_tools.rgb_cube_detector import CubeDetection, RgbCubeDetector


class CubeCandidateStabilityTest(unittest.TestCase):
    @staticmethod
    def detection(timestamp: float) -> CubeDetection:
        return CubeDetection(
            status="tracking",
            center_px=(120.0, 80.0),
            size_px=32.0,
            confidence=0.9,
            timestamp=timestamp,
            color="black",
        )

    def test_stable_frames_require_consecutive_observations(self) -> None:
        detector = RgbCubeDetector()

        candidates = []
        for frame in range(6):
            detection = self.detection(float(frame))
            candidates, selected = detector._update_candidate_tracks(
                [detection], float(frame), detection
            )

        self.assertEqual(selected, "cube-1")
        self.assertEqual(candidates[0]["stable_frames"], 6)
        self.assertEqual(candidates[0]["observed_frames"], 6)

        candidates, _ = detector._update_candidate_tracks([], 6.0, None)
        self.assertEqual(candidates[0]["status"], "occluded")
        self.assertEqual(candidates[0]["stable_frames"], 0)
        self.assertEqual(candidates[0]["observed_frames"], 6)

        detection = self.detection(7.0)
        candidates, selected = detector._update_candidate_tracks(
            [detection], 7.0, detection
        )
        self.assertEqual(selected, "cube-1")
        self.assertEqual(candidates[0]["stable_frames"], 1)
        self.assertEqual(candidates[0]["observed_frames"], 7)


if __name__ == "__main__":
    unittest.main()
