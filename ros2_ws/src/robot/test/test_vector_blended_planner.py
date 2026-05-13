from __future__ import annotations

import sys
import unittest
from pathlib import Path
import math
import numpy as np

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from robot.path_planner import VectorBlendedPlanner

class VectorBlendedPlannerTests(unittest.TestCase):
    def test_navigate_straight_ahead_no_obstacles(self) -> None:
        planner = VectorBlendedPlanner(
            lookahead_dist=150.0,
            max_angular=2.0,
            repulsion_gain=500.0,
            repulsion_range=400.0,
            goal_tolerance=20.0,
        )
        empty = np.empty((0, 2))
        pose = (0.0, 0.0, 0.0)
        waypoints = [(500.0, 0.0)]
        max_linear = 200.0

        linear, angular = planner.compute_velocity(pose, waypoints, empty, max_linear)

        self.assertGreater(linear, 0.0)
        self.assertAlmostEqual(angular, 0.0, places=2)

    def test_obstacle_directly_ahead_causes_repulsion(self) -> None:
        planner = VectorBlendedPlanner(
            lookahead_dist=200.0,
            max_angular=2.0,
            repulsion_gain=1000.0,
            repulsion_range=400.0,
        )
        # Obstacle at (100, 0) in robot frame (directly ahead)
        obstacles_r = np.array([[100.0, 0.0]])
        pose = (0.0, 0.0, 0.0)
        waypoints = [(500.0, 0.0)]
        max_linear = 200.0

        linear, angular = planner.compute_velocity(pose, waypoints, obstacles_r, max_linear)

        # Repulsive vector should point back (-1, 0)
        # Attraction vector is (+1, 0)
        # Blended vector will be (1 - gain/dist, 0)
        # If gain/dist > 1, total vector points back, steering angle = pi
        # Linear speed should be 0 because cos(pi) = -1, max(0, -1) = 0
        self.assertAlmostEqual(linear, 0.0, places=2)
        # Steering angle is pi, so angular should be at max_angular (2.0)
        self.assertAlmostEqual(abs(angular), 2.0, places=2)

    def test_obstacle_to_the_right_causes_left_turn(self) -> None:
        planner = VectorBlendedPlanner(
            lookahead_dist=200.0,
            max_angular=2.0,
            repulsion_gain=1000.0,
            repulsion_range=400.0,
        )
        # Obstacle at (100, -50) in robot frame (ahead and slightly right)
        obstacles_r = np.array([[100.0, -50.0]])
        pose = (0.0, 0.0, 0.0)
        waypoints = [(500.0, 0.0)]
        max_linear = 200.0

        linear, angular = planner.compute_velocity(pose, waypoints, obstacles_r, max_linear)

        # Repulsive vector from (100, -50) to (0,0) is (-100, 50)
        # Attractive vector is (1, 0) normalized or similar.
        # Blended vector should have positive Y (pointing left).
        self.assertGreater(angular, 0.0)

    def test_stops_at_goal(self) -> None:
        planner = VectorBlendedPlanner(goal_tolerance=20.0)
        empty = np.empty((0, 2))
        pose = (490.0, 0.0, 0.0)
        waypoints = [(500.0, 0.0)]
        max_linear = 200.0

        linear, angular = planner.compute_velocity(pose, waypoints, empty, max_linear)

        self.assertEqual(linear, 0.0)
        self.assertEqual(angular, 0.0)

if __name__ == "__main__":
    unittest.main()
