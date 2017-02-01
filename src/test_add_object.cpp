#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_add_object");
  ros::NodeHandle nh;

  ros::AsyncSpinner spin(1);
  spin.start();

  moveit::planning_interface::PlanningSceneInterface current_scene;
  moveit_msgs::CollisionObject box;

  // ある程度待たないとダメっぽい
  sleep(5.0);

  box.id = "box";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.5;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.4;
  pose.position.y = 0.4;
  pose.position.z = 1.0;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(pose);
  box.operation = box.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(box);

  current_scene.addCollisionObjects(collision_objects);
  ROS_INFO("Add the scene.");

  ros::shutdown();

  return 0;
}
