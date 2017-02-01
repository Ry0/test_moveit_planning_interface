# test_moveit_planning_interface
planningを繰り返しで実行する．動きを確認するためのパッケージ．
## 実行方法

```bash
roslaunch motoman_gazebo sia5_empty_world.launch gui:=false
roslaunch motoman_moveit sia5_moveit_planning_execution.launch 
rosrun test_moveit_planning_interface test_moveit_plan
```

```bash
roslaunch motoman_gazebo sia5_empty_world.launch gui:=false
roslaunch motoman_moveit sia5_moveit_planning_execution.launch 
rosrun test_moveit_planning_interface test_add_object
```