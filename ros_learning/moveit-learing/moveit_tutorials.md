# 记录按照moveit noetic教程学习的过程

## rviz过程

通过rviz的插件引入**MotionPlanning**插件。
![alt text](image.png)
可以发现有图中的很多的设置对象。对图中的设置解释一下：

1. 设置坐标系 Global Options中设置fixed_frame为我们机器人的基坐标系。
2. （以下为对MotionPlanning中的设置）设置机器人订阅话题 robot_description
3. 设置运动规划场景 planning scene topic
4. 设置轨迹话题 trajectory topic
5. 设置规划请求 planning request设置规划组Planning Group

### 对机器人的可视化演示
![alt text](image-1.png)

1. 在scense robot中选择是否展示机器人的场景状态 show robot visual
2. 在planned path中选择是否展示机器人show robot visual（白色实体机器人）
3. 在planning request中选择是否展示起始结束状态 Query start\goal state（绿色为起始状态、橙色为目标状态）

在joints部分可以保持末端不变而改变关节空间参数，实现零空间探索。那么什么时零空间探索呢，“Null Space Exploration”（零空间探索）是机器人运动规划和控制中一个非常核心且高级的概念，主要用于冗余机器人。在保持机器人末端执行器（手/夹爪）位置和姿态完全不变的情况下，调整机器人内部关节的角度。


![alt text](image-2.png)
可以通过鼠标拖动来设置start\goal state，也可以通过下拉folder来选择状态，主要分为以下几种：

1. The current state
2. The previous state(前一次规划的状态)
3. A randomly sampled state
4. A named state of the selected planning group, as defined in the srdf file of the robot.(在srdf中定义的状态)

### 对规划的状态进行回顾

在panels中选择MotionPlanning-Slider，添加到rviz中。
![alt text](image-3.png)

拖动滑杆可以看到规划中的状态，点击play可以冲滑杆位置，开始演示路径执行的过程。


### 设置速度、加速度
![alt text](image-4.png)

默认速度、加速度设置的为机器人最大值的10%，我们可以对这个缩放因子进行调整，或者修改我们机器人的最大速度、加速度的设置（在moveit_config中的joint_limit.yaml）

### 规划过程的分步执行
![alt text](image-5.png)
主要借助于moveit_visual_tools工具，在panels中选择RvizVisualToolsGui，将插件加入到rviz中。

## C++接口

最简单的用户接口是通MoveGroupInterface class，**It provides easy to use functionality for most operations** that a user may want to carry out, specifically setting **joint or pose goals**, **creating motion plans**, **moving the robot**, **adding objects** into the environment and **attaching/detaching objects** from the robot.  This interface communicates over ROS topics, services, and actions to the MoveGroup Node.

就是通过该类(MoveGropInterface), 可以设置关节空间或者是笛卡尔空间的状态，可以求解相应的路径规划解，可以发布移动机器人的指令，可以添加障碍物，也可以添加附着物。
当然这只是一个接口的类，具体的这些功能的实现主要还是靠MoveGroup来实现，这个类主要是借助于话题、服务以及动作，向MoveGroup发送相应的请求。

```shell
# 执行以下两条指令
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials move_group_interface_tutorial.launch
```

先设置运动规划组planning groups/joint model group：
通过`moveit::planning_interface::MoveGroupInterface`设置接口对象，输入为规划组的名称，例如"aubo_i5"、"panda_arm"等
通过`moveit::planning_interface::PlanningSceneInterface`设置规划场景的对象
通过原始指针指向规划组的接口对象提升运行效率。
`const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);`

可视化设置：
通常使用rviz进行可视化，这里当然使用rviz相关的接口了，先初始化一个MoveItVisualTools对象
`moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");`
这里输入参数是参考坐标系的名称，这里选择的是机器人的基坐标系

```cpp
moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
visual_tools.deleteAllMarkers();// 清理旧标记
visual_tools.loadRemoteControl();// 加载远程控制 与rviz的GUI进行交互

// 设置一个三维矩阵来表示位置
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
text_pose.translation().z() = 1.0;
// 将文字放置到rviz中 位置 文字 白色 超大
visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

// 将设置发布到rviz上
visual_tools.trigger();
```

运动规划演示

```cpp
// 在rviz上画一个坐标轴
visual_tools.publishAxisLabeled(target_pose1, "pose1");
// 显示文字提示
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
// 绘制路径规划的轨迹
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
// 将以上的指令序列发送
visual_tools.trigger();
// 程序运行到这里会死锁 等待GUI next
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 规划相关的操作

**设置规划组、规划场景**
```cpp
static const std::string PLANNING_GROUP = "panda_arm";
moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
```

**设置笛卡尔状态（位置+姿态）**
```cpp
geometry_msgs::Pose target_pose1;
target_pose1.orientation.w = 1.0;
target_pose1.position.x = 0.28;
target_pose1.position.y = -0.2;
target_pose1.position.z = 0.5;
move_group_interface.setPoseTarget(target_pose1);
```

**设置关节空间状态（一组关节角度）**
```cpp
// 该指针包含 当前位置、速度、加速度
moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
std::vector<double> joint_group_positions;
current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians
move_group_interface.setJointValueTarget(joint_group_positions);
```

**设置运动参数(设置最大速度\加速度 缩放因子)**
```cpp
move_group_interface.setMaxVelocityScalingFactor(0.05);
move_group_interface.setMaxAccelerationScalingFactor(0.05);
```

**执行**
```cpp
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

// 轨迹执行（注意起始状态）
move_group_interface.execute(my_plan);
// 尝试向目标执行
move_group_interface.move();
```
**运动限制**
让机器人执行路径约束规划之前需要先指定路径约束和目标状态，下面就是定义的路径约束。
```cpp
moveit_msgs::OrientationConstraint ocm;
// 约束link 相对于那个坐标系
ocm.link_name = "panda_link7";
ocm.header.frame_id = "panda_link0";
ocm.orientation.w = 1.0;
// 定义每个轴允许晃动的角度误差范围
ocm.absolute_x_axis_tolerance = 0.1;
ocm.absolute_y_axis_tolerance = 0.1;
ocm.absolute_z_axis_tolerance = 0.1;
ocm.weight = 1.0;// 约束权重

moveit_msgs::Constraints test_constraints;
test_constraints.orientation_constraints.push_back(ocm);
move_group_interface.setPathConstraints(test_constraints);
```

**从预设状态开始规划**
```cpp
// 存储起始状态（关节、笛卡尔）
moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
geometry_msgs::Pose start_pose2;
start_pose2.orientation.w = 1.0;
start_pose2.position.x = 0.55;
start_pose2.position.y = -0.05;
start_pose2.position.z = 0.8;
// 运动学逆解 将笛卡尔空间的坐标转化为关节空间中的状态存储在start_state中
start_state.setFromIK(joint_model_group, start_pose2);
move_group_interface.setStartState(start_state);// 设置起始状态
move_group_interface.setPoseTarget(target_pose1);// 设置目标状态
move_group_interface.setPlanningTime(10.0);// 规划时间增长
```

**笛卡尔路径规划**
指定笛卡尔路径点（一个列表，存储则末端执行器经过的点）
需要注意的是笛卡尔运动相对于普通的关节规划，`move_group.setMaxVelocityScalingFactor(0.1);`，也就是我们无法通过这些函数限制速度加速度。原因在于computeCartesianPath 这是一个纯几何的计算，它生成的只是一串密集的空间坐标点（Waypoints），默认并没有包含“什么时间到达哪个点”的信息。也就是生成的路径一开始是没有“速度”概念的。所以我们需要手动给这条几何路径加上时间戳，也就是Trajectory Parameterization（轨迹参数化）。
```cpp
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(start_pose2);

geometry_msgs::Pose target_pose3 = start_pose2;

target_pose3.position.z -= 0.2;
waypoints.push_back(target_pose3);  // down

target_pose3.position.y -= 0.2;
waypoints.push_back(target_pose3);  // right

target_pose3.position.z += 0.2;
target_pose3.position.y += 0.2;
target_pose3.position.x -= 0.2;
waypoints.push_back(target_pose3);  // up and left

moveit_msgs::RobotTrajectory trajectory;
const double eef_step = 0.01;
double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);
ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

move_group_interface.execute(trajectory);
```

这里除了展示文字、规划轨迹、坐标轴还有路径
```cpp
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
for (std::size_t i = 0; i < waypoints.size(); ++i)
  visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
visual_tools.trigger();
```
```cpp
// 对笛卡尔路径进行修改
// 先生成几何路径（这一步只得到了一串点，还没有速度信息）
moveit_msgs::RobotTrajectory trajectory;
double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

// 2. 创建一个 RobotTrajectory 对象来处理数据
robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

// 引入时间参数化算法
trajectory_processing::IterativeParabolicTimeParameterization iptp;

// 设置你想要的速度和加速度比例 
// 比如：速度 0.1 (10%), 加速度 0.1 (10%)
bool success = iptp.computeTimeStamps(rt, 0.1, 0.1);

// 4. 将处理好（带时间戳）的轨迹放回 plan 中
rt.getRobotTrajectoryMsg(trajectory);
my_plan.trajectory_ = trajectory;

// 5. 执行
move_group.execute(my_plan);
```

**添加障碍物到规划场景**
定义障碍物信息，主要用到以下几个数据类型：第一个是`moveit_msgs::CollisionObject`定义的是障碍物的信息，包括**参考坐标系**(header.frame_id)、**名称**(id)、**形状**(primitives)、**位姿**(primitive_poses)、**操作类型**(operation )等。形状使用的数据类型是`shape_msgs::SolidPrimitive `，主要包括类型（type）、参数（dimensions）。位姿使用的数据类型是`geometry_msgs::Pose`
```cpp
moveit_msgs::CollisionObject collision_object;
collision_object.header.frame_id = move_group_interface.getPlanningFrame();
collision_object.id = "box1"; //名称

shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[primitive.BOX_X] = 0.1;
primitive.dimensions[primitive.BOX_Y] = 1.5;
primitive.dimensions[primitive.BOX_Z] = 0.5;

geometry_msgs::Pose box_pose;
box_pose.orientation.w = 1.0;
box_pose.position.x = 0.5;
box_pose.position.y = 0.0;
box_pose.position.z = 0.25;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;

std::vector<moveit_msgs::CollisionObject> collision_objects;
collision_objects.push_back(collision_object);

ROS_INFO_NAMED("tutorial", "Add an object into the world");
planning_scene_interface.addCollisionObjects(collision_objects);
```

**附着物路径规划**
模拟机器人抓取目标物体，携带物体进行运动规划。数据类型与障碍物相似，值得注意的是往往附着物的参考坐标系是末端执行器，当然对于眼在手外的抓取以基坐标为参考坐标系也是非常合理的。
```cpp
moveit_msgs::CollisionObject object_to_attach;
object_to_attach.id = "cylinder1";

shape_msgs::SolidPrimitive cylinder_primitive;
cylinder_primitive.type = primitive.CYLINDER;
cylinder_primitive.dimensions.resize(2);
cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
geometry_msgs::Pose grab_pose;
grab_pose.orientation.w = 1.0;
grab_pose.position.z = 0.2;

object_to_attach.primitives.push_back(cylinder_primitive);
object_to_attach.primitive_poses.push_back(grab_pose);
object_to_attach.operation = object_to_attach.ADD;
planning_scene_interface.applyCollisionObject(object_to_attach);

ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
move_group_interface.attachObject(object_to_attach.id, "panda_hand", { "panda_leftfinger", "panda_rightfinger" });
```

**场景中物品的移除**
障碍物亦或是附着物
```cpp
move_group_interface.detachObject(object_to_attach.id);
std::vector<std::string> object_ids;
object_ids.push_back(collision_object.id);
object_ids.push_back(object_to_attach.id);
planning_scene_interface.removeCollisionObjects(object_ids);
```