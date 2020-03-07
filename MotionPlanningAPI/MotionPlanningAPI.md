# 运行
第一个终端运行命令：
```
roslaunch panda_moveit_config demo.launch
```
第二个终端运行命令：
```
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
```
# 程序
## 准备工作
Planner在MoveIt中被设计成一个插件，使用这可以用ROS pluginlib interface来加载planner。在加载前，使用者需要两个对象，一个RobotModel和一个PlanningScene。因此，首先实例化一个RobotModelLoader对象，它会查询ROS parameter server中的机器人描述，并自动构建一个RobotModel。  
[RobotModelLoader](http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html)   
```cpp
  const std::string PLANNING_GROUP = "panda_arm";
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  ```
使用RobotModel初始化一个PlanningScene用来更新整个场景（包括机器人）的状态。
  ```cpp
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
```
设置关节到默认状态。
```cpp
  // Configure a valid robot state
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
```
之后通过名字构建一个loader来加载planner。这里会使用ROS pluginlib库。
```cpp
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;
  ```
这步应该是从ROS的参数服务器中获得一个Planning plugin的名字，然后加载这个planner。  
```diff
- !!!
- 没太搞清楚，留待以后理解。
```
  ```cpp
  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
```
## 可视化
还是用MoveItVisualTools包进行可视化。先做例行的可视化工作。
```cpp
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();
```
Remote control是一个方便操作的工具，允许使用在RViz中使用按键和键盘快捷键。
```cpp
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.loadRemoteControl();

  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
```
## 位姿目标
为panda机器人创建一个运动计划。指定末端执行器的期望位姿为输入。
```cpp
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.trigger();
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
```
用一个helper函数（在kinematic_constraints包中）来创建一个用于约束的请求。  
[kinematic_constraints包](http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132)
```cpp
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);
```
构建一个planning context用来处理场景、运动请求和运动响应。然后让这个contxt求解响应。
```cpp
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
```
## 可视化运动结果
用display_publisher向/move_group/display_planned_path话题发布轨迹消息。
```cpp
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* 显示轨迹 */
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  /* 把Planning scene中的状态设置成运动计划的最终状态  */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // 显示目标状态
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_1");
  visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```
## 关节空间目标
首先建立关节空间目标，push back进req的目标约束中。
```cpp
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
```
然后调用planner进行求解，之后把求解得到的轨迹显示出来。
```cpp
  /* Re-construct the planning context */
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  /* Call the Planner */
  context->solve(res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* Visualize the trajectory */
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);

  ```
  把轨迹发布出去。
  ```cpp
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  ```
  把Planning scene中的状态设置成运动计划的最终状态。
  ```cpp
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  ```
  这步显示目标状态。
  ```cpp
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_2");
  visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```
## 添加路径约束
在有路径约束的条件下规划姿态目标的运动。首先定义目标姿态并构建一个目标约束。
```cpp
  pose.pose.position.x = 0.32;
  pose.pose.position.y = -0.25;
  pose.pose.position.z = 0.65;
  pose.pose.orientation.w = 1.0;
  moveit_msgs::Constraints pose_goal_2 =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

  /* Now, let's try to move to this new pose goal*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_2);
```
对运动施加一个路径约束.
constructGoalConstraints函数应该是重载复用的，根据传入参数种类不同，改变约束的形式。
```diff
- !!!
- 有点没懂这块添加了一个四元数的约束是要干啥？原文是“asking for the end-effector to saty level”,保持水平？
```
```cpp
  geometry_msgs::QuaternionStamped quaternion;
  quaternion.header.frame_id = "panda_link0";
  quaternion.quaternion.w = 1.0;
  req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);
```
施加路径约束需要在机器人的工作空间中，以使planner明确机器人末端的可行位置。所以之后设置工作空间的参数。设置一个立方体的两个对角坐标即可。
```cpp
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
      req.workspace_parameters.min_corner.z = -2.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
      req.workspace_parameters.max_corner.z = 2.0;

  // Call the planner and visualize all the plans created so far.
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  /* Set the state in the planning scene to the final state of the last plan */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_3");
  visual_tools.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
```
