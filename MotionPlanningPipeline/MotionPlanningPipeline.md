motion planner可以用来规划运动路径。当我们需要提前处理运动规划请求或者处理已经规划完的路径时，可以使用planning pipeline，它可以使motion planner有提前处理和后处理阶段（Planning request adapters)。
# 运行

第一个终端运行命令：
```
roslaunch panda_moveit_config demo.launch
```
第二个终端运行命令：
```
roslaunch moveit_tutorials motion_planning_pipeline_tutorial.launch
```
# 程序
首先要创建两个对象：RobotModel和PlanningScene。之后加载planner。这里实例化了一个RobotModelLoader对象，它会查找ROS参数服务器中的机器人描述，然后自动构建RobotModel。  
[RobotModelLoader](http://docs.ros.org/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html)
```cpp
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));
```
使用RobotModelLoader对象构建一个Planning scene monitor,它可以见识Planning Scene差异，病实时更新到环境中。之后调用startSceneMonitor, startWorldGeometryMonitor 和startStateMonitor函数来启动监视器。
```cpp
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  /* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
                       the internal planning scene accordingly */
  psm->startSceneMonitor();
  /* listens to changes of world geometry, collision objects, and (optionally) octomaps
                                world geometry, collision objects and optionally octomaps */
  psm->startWorldGeometryMonitor();
  /* listen to joint state updates as well as changes in attached collision objects
                        and update the internal planning scene accordingly*/
  psm->startStateMonitor();
```
获取机器人模型，获取并追踪机器人当前状态。
```cpp
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();

  robot_state::RobotStatePtr robot_state(
      new robot_state::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  const robot_model::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("panda_arm");
```
最后构建PlanningPipeline对象。
```cpp
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
```
## 可视化
仍然使用MoveItVisualTools
```cpp
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script
     via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

  /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  visual_tools.trigger();

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
```
## 位置姿态目标
创建一个motion plan request，指定末端执行器的一个位姿作输入。
```cpp
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
```
使用kinematic_constraints包来构造一个运动学约束。
[kinematic_constraints](http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132)
```cpp
  req.group_name = "panda_arm";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);
```
在规划前，需要对Planning Scene创建一个只读的锁，保证不会影响整个场景。之后条用pipeline然后检查规划是否成功。
```cpp
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    planning_pipeline->generatePlan(lscene, req, res);
  }

  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
```
## 显示规划结果
```cpp
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```
