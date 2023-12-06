""" Lazy Probabilistic Roadmap (PRM) implementation. """

import numpy as np
import time
import warnings

from .planner_base import PathPlannerBase
from ..utils.motion import Path
from ..utils.search_graph import SearchGraph, Node
from ..utils.pose import Pose


class LazyPRMPlannerPolygon:
    """
    Polygon representation based implementation of PRM.
    """

    def __init__(
        self,
        max_connection_dist=2.0,
        max_nodes=50,
        world=None,
    ):
        """
        Creates an instance of a Lazy PRM planner.

        :param max_nodes: Maximum nodes sampled to build the PRM.
        :type max_nodes: int
        :param world: World object to use in the planner.
        :type world: :class:`pyrobosim.core.world.World`
        """
        # Parameters
        self.max_connection_dist = max_connection_dist
        self.max_nodes = max_nodes
        self.max_path_check = max_nodes
        self.world = world

        # construct a search graph in the configuration space
        self.construct_graph()

    def construct_graph(self):
        """
        construct a search graph with sample nodes without collision check.
        """ 
        t_start = time.time()
        self.graph = SearchGraph(
            color=[0, 0.4, 0.8], color_alpha=0.25, use_planner=True
        )

        x_bounds, y_bounds = self.world.get_bounds()
        for i in range(self.max_nodes):
            sample = self.sample_configuration(x_bounds, y_bounds)
            self.graph.add_node(Node(pose=sample))

        for node in self.graph.nodes:
            self.construct_edge(node)

        self.sampling_time = time.time() - t_start

    def sample_configuration(self, x_bounds, y_bounds):
        """
        Samples a random configuration within x y boundary without collision check.

        :return: a random pose within x y boundary
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        xmin, xmax = x_bounds
        ymin, ymax = y_bounds

        x = (xmax - xmin) * np.random.random() + xmin
        y = (ymax - ymin) * np.random.random() + ymin
        yaw = 2.0 * np.pi * np.random.random()
        pose = Pose(x=x, y=y, z=0.0, yaw=yaw)

        return pose

    def construct_edge(self, node):
        """
        Connect a node to all nodes within connection distance without collision check.
        """        
        for other in self.graph.nodes:
            if node == other:
                continue;
            distance = node.pose.get_linear_distance(other.pose, ignore_z=True)
            if self.max_connection_dist and (distance <= self.max_connection_dist):
                self.graph.add_edge(node, other)

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose or graph node.
        :type start: :class:`pyrobosim.utils.pose.Pose` /
            :class:`pyrobosim.utils.search_graph.Node`
        :param goal: Goal pose or graph node.
        :type goal: :class:`pyrobosim.utils.pose.Pose` /
            :class:`pyrobosim.utils.search_graph.Node`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        # Reset the path
        self.latest_path = Path()
        t_start = time.time()

        # Create the start and goal nodes
        start = Node(start, parent=None)
        goal = Node(goal, parent=None)
        self.graph.add_node(start)
        self.graph.add_node(goal)
        self.construct_edge(start)
        self.construct_edge(goal)

        # try to find a collision free path
        path = Path()
        i = 0
        while i < self.max_path_check:
            i += 1
            path = self.graph.find_path(start, goal)
            if self.is_path_collision_free(path):
                break;

        # if a collision free path is found, update the path to return
        if i != self.max_path_check:
            self.latest_path = path
            self.latest_path.fill_yaws()
        
        self.graph.remove_node(start)
        self.graph.remove_node(goal)
        self.planning_time = time.time() - t_start

        return self.latest_path

    def is_path_collision_free(self, path):
        """
        check if the path is collision free, if not, remove nodes in the graph that are not free

        :param path: Path from start to goal.
        :type class:`pyrobosim.utils.motion.Path`
        :return: True if all nodes in the path is collision free
        :rtype: bool
        """
        path_collion_free = True
        nodes_to_remove = set()
        for waypoint in path.poses:
            if self.world.check_occupancy(waypoint):
                path_collion_free = False
                for node in self.graph.nodes:
                    if waypoint == node.pose:
                        nodes_to_remove.add(node)
        for node in nodes_to_remove:
            self.graph.remove_node(node)        
        return path_collion_free

    def get_graphs(self):
        """
        Returns the graphs generated by the planner, if any.

        :return: List of graphs.
        :rtype: list[:class:`pyrobosim.utils.search_graph.SearchGraph`]
        """
        return [self.graph]


class LazyPRMPlanner(PathPlannerBase):
    """Factory class for Lazy Probabilistic RoadMap path planner."""

    def __init__(self, **planner_config):
        """
        Creates an instance of Lazy PRM planner.
        """
        super().__init__()

        self.impl = None

        if planner_config.get("grid", None):
            raise NotImplementedError("Grid based Lazy PRM is not supported. ")
        else:
            self.impl = LazyPRMPlannerPolygon(**planner_config)
            print(f"Sampling time : {self.impl.sampling_time}")

    def plan(self, start, goal):
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`
        :param goal: Goal pose.
        :type goal: :class:`pyrobosim.utils.pose.Pose`
        :return: Path from start to goal.
        :rtype: :class:`pyrobosim.utils.motion.Path`
        """
        self.latest_path = self.impl.plan(start, goal)
        print(f"Planning time : {self.impl.planning_time}")
        self.graphs = self.impl.get_graphs()
        return self.latest_path
