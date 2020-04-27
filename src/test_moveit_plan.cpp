#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros_colored_msg.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_moveit_plan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("manipulator");
  group.setPlannerId("PTP");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  for (int i = 0; i < 10; ++i)
  {
    // Set start state from the current pose
    group.setStartState(*group.getCurrentState());

    // Set a goal joints value
    // for a.scene
    // std::map<std::string, double> joints;
    // joints["joint_b"] = 0.32058177929787846;
    // joints["joint_e"] = 1.8350539569231312;
    // joints["joint_l"] = -1.5281191739279407;
    // joints["joint_r"] = -0.25001577247785445;
    // joints["joint_s"] = -1.9937642184382387;
    // joints["joint_t"] = -1.1872623791986276;
    // joints["joint_u"] = 0.10100470429649366;
    // group.setJointValueTarget(joints);

    // for b.scene
    // std::map<std::string, double> joints;
    // joints["joint_b"] = 1.670018164510486;
    // joints["joint_e"] = -0.5271197974997417;
    // joints["joint_l"] = 1.281747824875227;
    // joints["joint_r"] = 0.07279277864957656;
    // joints["joint_s"] = 0.08170077790963859;
    // joints["joint_t"] = -0.4973888051324167;
    // joints["joint_u"] = -0.39089672858583313;
    // group.setJointValueTarget(joints);

    // Set the random pose.
    group.setRandomTarget();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    ROS_GREEN_STREAM("Finish " << i + 1 << " loop");
    sleep(3);
  }

  ros::shutdown();
  return 0;
}
