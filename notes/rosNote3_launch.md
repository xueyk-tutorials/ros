# ROS笔记——launch

## 参考

http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams#Using_rosparam



## 参数

### arg



### param

http://wiki.ros.org/rospy/Overview/Parameter%20Server

value的类型有:

- integer: 整数类型
- boolean: bool类型
- double: 小数类型
- list: 集合列表类型
- map: 字典类型
- binary: 二进制数据类型



## rosparam

在大型编程中可能会涉及到很多参数配置，在ROS中可以将这些参数放到一个YAML文件，运行时加载至rosmaster的参数服务器中供所有节点访问。

### 命令行操作

```shell
$ rosparam list
$ rosparam get /turtlesim/background_g 
$ rosparam set /turtlesim/background_r 150
```



### 使用示例1

我们首先在工作空间下新建一个package命名为**learning_parameter**，也即在/catkin_ws/src下创建了一个learning_parameter文件夹，然后在package下创建config文件夹，用于放置.yaml文件，文件结构如下：

learning_parameter/

|--config/

​    |--turtle_param.yaml

|--src

​    |--parameter_test.cpp

**turtle_param.yaml**

```yaml
background_b: 255
background_g: 86
background_r: 69
rosdistro: 'melodic'
roslaunch:
  uris: {host_hcx_vpc__43763: 'http://hcx-vpc:43763/'}
rosversion: '1.14.3'
run_id: 077058de-a38b-11e9-818b-000c29d22e4d
```

**parameter_test.cpp**

我们可以通过如下C++代码来操作param的读取和设置。

```c++
int red, green, blue;
// 读取背景颜色参数
ros::param::get("/background_r", red);
ros::param::get("/background_g", green);
ros::param::get("/background_b", blue);
ROS_INFO("Get Backgroud Color[%d, %d, %d]", red, green, blue);
// 设置背景颜色参数
ros::param::set("/background_r", 255);
ros::param::set("/background_g", 255);
ros::param::set("/background_b", 255);
```



## roslaunch

ROS在启动时需要手动打开很多个终端，并且需要依次输入ros命令，非常不方便。可以通过roslaunch实现ROS的快速启动和配置。

我们可以在package/launch文件夹建立**.launch**文件：通过XML文件实现多节点的配置和启动（可自动启动ROS Master）。

参考文档：

http://wiki.ros.org/roslaunch/XML

### launch启动

启动launch文件的语法如下：

`roslaunch pkg_name file.launch`

- pkg_name为功能包名
- file.launch为launch文件名

### launch文件语法

launch文件的根元素都是采用<launch>标签定义。

#### 节点启动node

启动节点的代码格式为：`<node pkg="pkg_name" type="executable_name" name="node_name"/>`。其中：

- pkg：节点所在的功能包(package)名称
- type：节点的可执行程序名，如果是python节点，需要加.py后缀，如果是C/C++，则是编译生成的可执行程序名
- name：节点运行时的名称
- output、respawn、required、ns、args

例如：

```yaml
<launch>
	<!-- for C/C++ -->
    <node pkg="turtlesim" type="turtlesim_node" name="sim1" output="screen" />
    <node pkg="turtlesim" type="turtlesim_node" name="sim2" output="screen" />
    <!-- for python -->
    <node pkg="turtlesim" type="turtlesim_node.py" name="sim2" output="screen" />
</launch>
```

#### group

很多时候，你需要启动多个同样的实例程序，例如你希望在本地计算机启动多个*turtle*实例，但很显然这些实例程序都是一样的，而且是同一个node，只是启动了多次而且，那么如何将这些同名的node进行管理和区分呢？

我们可以使用分组的概念，将每个node划分到不同的*group*下面，实现方式很简单，在launch文件中，使用*group*标签即可新建一个组，在这个组下面启动的node，**ROS master会自动给其添加一级前缀名（名字就是group标签定义的namespace名称）**。

```xml
<group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
</group>
<group ns="turtlesim2">
   <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
</group>
```

也就是说，namespace的名称会作为第一级**路径**名称添加到每个node的前头，这样就能够定位区分不同的node，例如，第一个node的topic就可以使用**/turtlesim1/cmd_vel**来访问了。

#### 文件参数param

设置ROS系统运行中的参数，存储在参数服务器中。代码格式为：

`<param name="output_frame" value="odom" type="int"/>`

- name：参数名
- value：参数值
- type：类型



```xml
<param name="drone_type" value="$(arg drone_type)" type="string"/>

<node pkg="flocking" type="drone.py" name="drone1" output="screen">
    <param name="drone_id" value="1" type="int"/>
</node>
<node pkg="flocking" type="drone.py" name="drone2" output="screen">
    <param name="drone_id" value="2" type="int"/>
</node>
```



根据param的位置:

1. 如果参数在node前声明

   那么对后续所有node都生效，在node文件中，格式如下：

   ```shell
   self.drone_id     = rospy.get_param('drone_id',    0)
   ```

   

2. 如果参数在node中声明

   那么仅仅对当前node有效，在node文件中，获取参数时需要添加`~`前缀，格式如下：

   ```shell
   self.drone_id     = rospy.get_param('~drone_id',    0)
   ```

   

   

加载参数文件的多个参数代码格式为：

`<rosparam file="params.yaml" command="load" ns="params" />`

#### 设置launch内部参数

launch内部参数顾名思义就是只能在launch文件内部使用的参数，代码格式为：

`<arg name="arg_name" default=“arg_value" />`

- name：参数名
- value：参数值

调用方式为如下：

```
<param name="foo" value="$(arg arg_name)" />
<node name="node" pkg="package" type="type" args="$(arg arg_name)" />
```

#### 重映射remap

重映射ROS计算图资源的名字，代码格式如下：

`<remap from="/turtlebot/cmd_vel" to="/cmd_vel" />`

- from：原命名
- to：映射之后的命名

#### 嵌套include

包含其他launch文件，类似C语言的头文件包含，代码格式如下：

`<include file="$(dirname)/other.launch" />`

- file：包含的其他launch文件路径

```xml
<?xml version="1.0"?>
<launch>
    <include file="$(find driver_flightcontroller)/launch/driver_test.launch">
    </include>
</launch>
```

#### 参数设置rosparam

对于大型工程来说，有很多参数需要配置，完全写在*launch*文件中不同合适，这时候可以将这些参数使用**.yaml**文件进行存储和描述，并且可以使用<rosparam>标签引用**yaml**文件，启动后这些参数会被放置到参数服务器中。

**workspace**

```
|--onboard_app/
  |--config/
    |--param.yaml
  |--launch/
    |--test.launch
```

**yaml**

```yaml
# Common configuration for onboard application
#
pod:
  uart_name: "/dev/ttyS0"
  uart_baudrate: 115200
```

**launch**

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="onboard_app" type="pod_app" name="pod_app" output="screen" />
    <rosparam file="$(find onboard_app)/config/param.yaml" command="load" />
</launch>
```

**how to use param defined in yaml**

```c++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_pod_app");
    ros::NodeHandle nh;

    nh.param("/pod/uart_name", name, std::string("/default"));
    nh.param("/gcs/gcs_ip", gcs_ip, std::string("/default"));
    ros::spin();
    return 0;
}
```



### launch文件参数传递

节点中如果使用launch文件中的参数，可以有三种方式：

1、ros::param::get()

2、nh.getParam()

3、nh.param<>()

### 例程

#### 参数使用

1、文件`alex.launch`

```xml
<?xml version="1.0" encoding="utf-8"?>
<launch>
  <arg name="launch_prefix" default=""/>
  <param name="video_type" value="camera" />

  <node pkg="alex_ros" type="alex_node" name="alex_node" output="screen" launch-prefix="$(arg launch_prefix)">
    <param name="video_topic" value="/camera/rgb/image_raw" />
    <param name="video_address"  value="/home/dji/Desktop/videos/lby_xiongmaota1.mp4" />
  </node>
</launch>

```

launch文件定义的参数会加载至ROS参数服务器，以上launch文件运行后，你可以使用`rosparam list`查看增加了如下参数：

```
/video_type
/alex_node/video_topic
/alex_node/video_address
```

2、文件`alex_node.cpp`

```c++
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_publisher");
    //添加参数"~"指定私有命名空间：nh的namespace为/alex_node
    ros::NodeHandle nh("~");
    std::string video_type;
    std::string video_topic;
    std::string video_addr;
    
    nh.getParam("/video_type", video_type);
    nh.param<std::string>("video_topic", video_topic, std::string("/camera/rgb/image_raw"));
    nh.param<std::string>("video_address", video_addr, std::string("/home/dji/Desktop/videos/mAP40.mp4"));

}
```

#### group实例多个node

使用group可以实例化多个node，例如启动多个小海龟节点，每个节点划分到不同group中。

```xml
<?xml version="1.0" encoding="utf-8"?>
<launch>
    <group ns="turtlesim1">
    	<node pkg="turtlesim" name="sim" type="turtlesim_node"/>
    </group>
    <group ns="turtlesim2">
        <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
    </group>
</launch>
```

查看输出如下：

```
alex@alex:~$ roslaunch px4 multi_turtle.launch 
... logging to /home/alex/.ros/log/5a80d71c-1219-11eb-9df8-fc7774f4aa0b/roslaunch-alex-4609.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://alex:41865/

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.7

NODES
  /turtlesim1/
    sim (turtlesim/turtlesim_node)
  /turtlesim2/
    sim (turtlesim/turtlesim_node)

auto-starting new master
process[master]: started with pid [4620]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 5a80d71c-1219-11eb-9df8-fc7774f4aa0b
process[rosout-1]: started with pid [4631]
started core service [/rosout]
process[turtlesim1/sim-2]: started with pid [4634]
process[turtlesim2/sim-3]: started with pid [4635]

```

