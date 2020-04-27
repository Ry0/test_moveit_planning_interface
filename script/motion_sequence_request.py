#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3
from geometry_msgs.msg import Pose
import moveit_commander
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes
from moveit_msgs.msg import WorkspaceParameters, DisplayTrajectory
from moveit_msgs.msg import MotionSequenceItem, MotionPlanRequest, MotionSequenceRequest, MoveGroupSequenceAction, MoveGroupSequenceGoal
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.srv import GetMotionSequence
from math import pi
import actionlib

from shape_msgs.msg import SolidPrimitive


def create_basic_mp_joint_request(joint_names, joint_values, planner_id):
    motion_plan_request = MotionPlanRequest()
    motion_plan_request.group_name = move_group
    motion_plan_request.num_planning_attempts = 1
    motion_plan_request.allowed_planning_time = 5.0
    motion_plan_request.workspace_parameters = WorkspaceParameters()
    motion_plan_request.max_velocity_scaling_factor = 0.5
    motion_plan_request.max_acceleration_scaling_factor = 0.5
    motion_plan_request.planner_id = planner_id

    joint_constraints = []
    for i in range(len(joint_names)):
        joint_con = JointConstraint()
        joint_con.joint_name = joint_names[i]
        joint_con.position = joint_values[i]
        joint_con.tolerance_above = 0.0001
        joint_con.tolerance_below = 0.0001
        joint_con.weight = 0.0001
        joint_constraints.append(joint_con)
    constraint = Constraints()
    constraint.joint_constraints = joint_constraints
    constraints = [constraint]
    motion_plan_request.goal_constraints = constraints
    return motion_plan_request


def create_basic_mp_position_request(
        planning_frame,
        link_name,
        target_point_offset,
        planner_id):
    motion_plan_request = MotionPlanRequest()
    motion_plan_request.group_name = move_group
    motion_plan_request.num_planning_attempts = 1
    motion_plan_request.allowed_planning_time = 5.0
    motion_plan_request.workspace_parameters = WorkspaceParameters()
    motion_plan_request.max_velocity_scaling_factor = 0.5
    motion_plan_request.max_acceleration_scaling_factor = 0.5
    motion_plan_request.planner_id = planner_id

    position_constraints = []
    position_constraint = PositionConstraint()
    header = std_msgs.msg.Header()
    header.frame_id = planning_frame
    position_constraint.header = header
    position_constraint.link_name = link_name
    position_constraint.target_point_offset = target_point_offset
    position_constraints = [position_constraint]
    constraint = Constraints()
    constraint.position_constraints = position_constraints
    constraints = [constraint]
    motion_plan_request.goal_constraints = constraints
    return motion_plan_request


rospy.init_node('move_group_client')
# action_topic = '/move_group'
# client = actionlib.SimpleActionClient(action_topic, MoveGroupAction)

move_group = 'manipulator'
group = moveit_commander.MoveGroupCommander(move_group)
robot = moveit_commander.RobotCommander()
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    DisplayTrajectory,
    queue_size=20)

planning_frame = group.get_planning_frame()
print('planning_frame:', planning_frame)

eef_link = group.get_end_effector_link()
print('eef_link:', eef_link)

tolerance = 0.01

joint_names = group.get_joints()
random_joint_values = group.get_random_joint_values()
mp_req_1 = create_basic_mp_joint_request(
    joint_names, random_joint_values, "PTP")

random_joint_values = group.get_random_joint_values()
mp_req_2 = create_basic_mp_joint_request(
    joint_names, random_joint_values, "PTP")

# pose_goal = Vector3()
# pose_goal.x = 0.4
# pose_goal.y = 0.1
# pose_goal.z = 0.1
# # random_pose = group.get_random_pose()
# mp_req_1 = create_basic_mp_position_request(
#     planning_frame, eef_link, pose_goal, "LIN")

# pose_goal = Vector3()
# pose_goal.x = 0.1
# pose_goal.y = 0.1
# pose_goal.z = 0.2
# # random_pose = group.get_random_pose()
# mp_req_2 = create_basic_mp_position_request(
#     planning_frame, eef_link, pose_goal, "LIN")

motion_sequence_request = MotionSequenceRequest()
item_1 = MotionSequenceItem()
item_1.blend_radius = 0
item_1.req = mp_req_1

item_2 = MotionSequenceItem()
item_2.blend_radius = 0
item_2.req = mp_req_2

motion_sequence_request = MotionSequenceRequest()
motion_sequence_request.items = [item_1, item_2]

rospy.wait_for_service('plan_sequence_path')
try:
    service_call = rospy.ServiceProxy('plan_sequence_path', GetMotionSequence)
    result = service_call(motion_sequence_request)

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory = result.response.planned_trajectories
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
except rospy.ServiceException as e:
    print ("Service call failed: %s" % e)
