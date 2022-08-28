# urdf基础

## 参考

http://wiki.ros.org/urdf/Tutorials


http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch



### 一些必要的ROS包

```shell
sudo apt-get install ros-melodic-joint-state-publisher-gui
```

控制包

```shell
$ sudo apt-get install ros-melodic-position-controllers
$ sudo apt-get install ros-melodic-joint-state-controller
$ sudo apt-get install ros-melodic-velocity-controllers
$ sudo apt-get install ros-melodic-effort-controllers

```







# urdf开发流程

## 基础流程

使用urdf文件定义机器人模型

通过launch文件设置`robot_description`参数传入urdf文件

启动`joint_state_publisher_gui`

启动`robot_state_publisher`

启动`rviz`

## gazebo流程





# 标签讲解

参考的代码为：https://github.com/ros/urdf_tutorial/tree/master



只能有一个root link

所有的其他link都必须通过joint挂载到root link

### link

link用于定义一个坐标系并在该坐标系下设计一个机器人的部件！

- origin

  在当前坐标系下，对所有部件进行变换；

  1）rpy表示将所有部件进行旋转，即roll pitch yaw，分别绕x y z轴转；

  2）xyz表示将所有部件进行平移；

  **这里的旋转平移是指机器人部件在本坐标系下的旋转和平移！**

- geometry

  添加实体，有多种选择，例如：

  1）box，长方体

  2）mesh，网格体，指定`.dae`格式文件；



```shell
  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57 0" xyz="0 0 -0.0"/>
    </visual>
  </link>

  <link name="left_gripper">
    <visual>
      <origin rpy="0.0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>
  
```



### joint

#### 定义

- name

  关节名称

- type

  关节类型，例如：

  1）fixed：固定关节，不允许运动的特殊关节（常用）

  2）continuous：旋转关节，可以绕单轴无限旋转（常用）

  3）revolute：旋转关节，类似于 continues，但是有旋转角度限制

  4）prismatic：滑动关节，沿某一轴线移动的关节，有位置极限

  5）planer：平面关节，允许在平面正交方向上平移或旋转

  6）floating：浮动关节，允许进行平移、旋转运动

#### 常用子标签

- origin

  定义以parent坐标系为参考，child坐标系的变换关系

  1）rpy表示欧拉角，对应$R_c^p$；

  2）xyz表示child原点在parent下的坐标，$t_c^p$；

  这里的旋转平移是指child坐标系在parent坐标系下的旋转平移！

- axis

  当type=continuous时，定义旋转轴，例如`<axis xyz="0 1 0"/>`代表绕y轴旋转。



示例

```xml
  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>
```



# xacro

xacro是为了解决在编写urdf文件过程中，出现的标签内容重复，但又不得不去写。一些值的计算有依赖的问题。为了和urdf文件区分，我们定义的模型文件名后缀为.xacro。

从功能的角度来说，xacro提供了属性定义，数学运算，条件判断和宏定义等功能。

xacro 提供了可编程接口，类似于计算机语言，包括变量声明调用、函数声明与调用等语法实现。在使用 xacro 生成 urdf 时，根标签`robot`中必须包含命名空间声明:`xmlns:xacro="http://wiki.ros.org/xacro"`

# gazebo

## gazebo中加载urdf

### urdf文件

urdf文件必须添加的标签如下：

inertial，collision

- 如果物体在gazebo中来回振动，说明inertial中定义的转动惯量和物体尺寸不一致，可以改变ixx、iyy、izz的值，例如尺寸和重量变大了之后，把转动惯量也要增加；

- 如果物体嵌入了地面，应该是collision中形状大小设置不合适，如果一个机器人的轮子几何尺寸直径为0.5，但该轮子下的collision设置的几何尺寸为0.3，就会陷入gazebo中地面0.2m；

### xacro文件



### launch文件

```xml
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>
    
  <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" args="-file $(find learn_urdf)/urdf/robot01_6gazebo.urdf -urdf -z 1 -model robot_description" output="screen"/>

</launch>
```

# 仿真常用节点说明

## joint_state_publisher_gui

- 功能

  发布命令用于控制机器人joint的状态。

- package

  joint_state_publisher_gui

- 运行

  ```shell
  $ rosrun joint_state_publisher_gui joint_state_publisher_gui
  ```

一般在调试时我们都会希望使用一个GUI界面控制机器人，`joint_state_publisher_gui`会自动根据机器人的关节定义配置GUI内容，获取joint name并生成简单的控件！需要注意的是，如果joint的type是`fixed`那么则代表joint无法控制，则不会生成对应控件！

使用注意：

1. 不要与joint_state_publisher同时使用！不然造成两个节点发布不同的控制指令，造成机器人关节状态不停切换！

## robot_state_publisher



### spawn_model

用于在gazebo中加载模型



# 示例

## launch文件示例

### 在rviz中显示

#### 节点robot_state_publisher

参考：http://wiki.ros.org/robot_state_publisher

解释：这个节点主要作用是加载urdf描述文件生成机器人，并且订阅`joint_states` ([sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)) 消息，更新机器人动力学状态并发布结果至tf。并且rviz可通过加载RobotModel获取这个机器人的实时状态。

说明：

- 其中`joint_states` 的发布者一般由开发者来实现，也就是开发者写的程序中必须有个发布([sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)) 消息至话题joint_states。
- 一般调试的时候，也可以使用ROS自带的joint_state_publisher包和joint_state_publisher_gui包下的`joint_state_publisher`节点。



```shell
<launch>

  <arg name="model" default="$(find urdf_tutorial)/urdf/01-myfirst.urdf"/>
  <arg name="gui" default="true" />
  <arg name="rvizconfig" default="$(find urdf_tutorial)/rviz/urdf.rviz" />

  <param name="robot_description" command="$(find xacro)/xacro $(arg model)" />

  <node if="$(arg gui)" name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
  <node unless="$(arg gui)" name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />

</launch>

```





# PX4仿真



mavros_posix_sitl.launch

### posix_sitl.launch



定义模型

sitl_gazebo/models/iris/iris.sdf

包括了

- 电机插件（libgazebo_motor_model.so）

- imu插件（libgazebo_imu_plugin.so）
- 等等

机载gazebo世界

加载gazebo模型





相机相关模型

iris_stereo_camera.sdf

stereo_camera.sdf

