"""
Utilities to convert between standalone pyrobosim objects and 
ROS representations (messages, services, etc.).
"""

from transforms3d.euler import euler2quat, quat2euler
import geometry_msgs.msg

import pyrobosim_msgs.msg as ros_msgs
from pyrobosim.utils.pose import Pose
import pyrobosim.planning.actions as acts


def pose_from_ros(msg):
    """
    Converts ROS pose message to a pyrobosim pose.

    :param act: ROS message.
    :type act: :class:`geometry_msgs.msg.Pose` 
    :return: Pose object
    :rtype: :class:`pyrobosim.utils.pose.Pose`
    """
    eul = quat2euler((msg.orientation.w, msg.orientation.x,
                      msg.orientation.y, msg.orientation.z))
    return Pose.from_list(
        [msg.position.x, msg.position.y, msg.position.z, eul[2]])


def pose_to_ros(pose):
    """
    Converts a pyrobosim Pose to a ROS message.

    :param act: Pose object.
    :type act: :class:`pyrobosim.utils.pose.Pose`    
    :return: ROS message.
    :rtype: :class:`geometry_msgs.msg.Pose`
    """
    pose_msg = geometry_msgs.msg.Pose()
    if pose is not None:
        pose_msg.position.x = pose.x
        pose_msg.position.y = pose.y
        pose_msg.position.z = pose.z
        quat = euler2quat(0, 0, pose.yaw)
        pose_msg.orientation.w = quat[0]
        pose_msg.orientation.x = quat[1]
        pose_msg.orientation.y = quat[2]
        pose_msg.orientation.z = quat[3]
    return pose_msg


def get_entity_name(entity):
    """
    Gets the name of an entity, or if a string is specified, gets the string itself.

    :param entity: Entity from which to extract the name
    :type entity: Entity, or str
    :return: Entity name.
    :rtype: str
    """
    if entity is None:
        return ""    
    elif isinstance(entity, str):
        return entity
    else:
        return entity.name


def goal_specification_from_ros(msg, world):
    """
    Uses a world object to resolve a GoalSpecification message to a 
    list of goal literals for task and motion planning.
    
    :param msg: ROS message.
    :type msg: :class:`pyrobosim_msgs.msg.GoalSpecification`
    :param world: World object to use to resolve literals.
    :type world: :class:`pyrobosim.core.world.World`
    :return: List of literals describing a goal specification.
    :rtype: list[tuple]
    """
    goal_literals = []
    for pred_msg in msg.predicates:
        pred = [pred_msg.type]

        # For each argument in the goal predicate, try resolve it to a real entity
        # in the world. If so, use the entity itself. Otherwise, use the name.
        for arg in pred_msg.args:
            entity = world.get_entity_by_name(arg)
            if entity:
                pred.append(entity)
            else:
                pred.append(arg)

        goal_literals.append(tuple(pred))

    return goal_literals


def task_action_from_ros(msg):
    """ 
    Converts a TaskAction ROS message to a TaskAction object.
    
    :param msg: ROS message.
    :type msg: :class:`pyrobosim_msgs.msg.TaskAction`
    :return: Task action object.
    :rtype: :class:`pyrobosim.planning.actions.TaskAction`
    """
    if not isinstance(msg, ros_msgs.TaskAction):
        raise Exception("Input is not a TaskAction ROS message")
    return acts.TaskAction(msg.type, object=msg.object, room=msg.room,
                 source_location=msg.source_location, target_location=msg.target_location,
                 pose=msg.pose, cost=msg.cost)


def task_action_to_ros(act):
    """ 
    Converts a TaskAction object to a TaskAction ROS message.

    :param act: Task action object.
    :type act: :class:`pyrobosim.planning.actions.TaskAction`    
    :return: ROS message.
    :rtype: :class:`pyrobosim_msgs.msg.TaskAction`
    """
    if not isinstance(act, acts.TaskAction):
        raise Exception("Input is not a TaskAction object")
    
    act_msg = ros_msgs.TaskAction(type=act.type)
    act_msg.object = get_entity_name(act.object)
    act_msg.room = get_entity_name(act.room)
    act_msg.source_location = get_entity_name(act.source_location)
    act_msg.target_location = get_entity_name(act.target_location)
    act_msg.pose = pose_to_ros(act.pose)    
    if act.cost:
        act_msg.cost = float(act.cost)

    return act_msg


def task_plan_from_ros(msg):
    """
    Converts a TaskPlan ROS message to a TaskPlan object.
    
    :param msg: ROS message.
    :type msg: :class:`pyrobosim_msgs.msg.TaskPlan`
    :return: Task plan object.
    :rtype: :class:`pyrobosim.planning.actions.TaskPlan`
    """
    if not isinstance(msg, ros_msgs.TaskPlan):
        raise Exception("Input is not a TaskPlan ROS message")
    actions = [task_action_from_ros(act_msg) for act_msg in msg.actions]
    return acts.TaskPlan(actions=actions)


def task_plan_to_ros(plan):
    """
    Converts a TaskPlan object to a TaskPlan ROS message.
    
    :param plan: Task plan object.
    :type plan: :class:`pyrobosim.planning.actions.TaskPlan`    
    :return: ROS message.
    :rtype: :class:`pyrobosim_msgs.msg.TaskPlan`
    """
    if not isinstance(plan, acts.TaskPlan):
        raise Exception("Input is not a TaskPlan object")
    act_msgs = [task_action_to_ros(act) for act in plan.actions]
    return ros_msgs.TaskPlan(actions=act_msgs, cost=plan.total_cost)
