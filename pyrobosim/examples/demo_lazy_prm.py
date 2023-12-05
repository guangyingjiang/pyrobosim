#!/usr/bin/env python3
import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation import PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Load a test world.
world_file = os.path.join(get_data_folder(), "test_world.yaml")
world = WorldYamlLoader().from_yaml(world_file)


def test_lazy_prm():
    """Creates a Lazy PRM planner and plans"""
    planner_config = {
        "world": world,
        "max_nodes": 1000,
        "max_connection_dist": 0.5
    }
    lazyPRM = PathPlanner("lazy_prm", **planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)

    robot = world.robots[0]
    robot.set_pose(start)
    robot.set_path_planner(lazyPRM)
    result = robot.plan_path(start, goal)
    lazyPRM.info()


if __name__ == "__main__":
    test_lazy_prm()
    start_gui(world)
