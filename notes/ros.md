# ROS

**Author:** alex

**Date:**2020/05/25



## ROS简介

ROS的目标是提高机器人研发中的软件复用率

### 通信机制

松耦合分布式通信

### 生态系统

发行版（Distribution）：ROS发行版包括一系列带有版本号、可以直接安装的功能包。

软件源（Repository）：ROS依赖于共享网络上的开源代码，不同的组织结构可以开发或者共享自己的机器人软件。是存放编译好的安装文件，可以apt-get install进行安装你需要的功能包。

ROS wiki：记录ROS信息文档的主要论坛。

ROS Answers：咨询ROS相关问题的网站

Blog：发布ROS社区新闻、图片、视频等，http://www.ros.org/news

### 历史

![ros_history](https://gitee.com/bpnotes/pic-museum/raw/master/pictures/ros_history.PNG)

## ROS核心概念

### 节点与节点管理器

节点Node就是一个执行单元：

- 执行具体任务的进程、独立运行的可执行文件；
- 不同节点可以使用不同的编程语言，可分布式运行在不同主机；
- 节点在系统中的名称必须是唯一的。

节点管理器ROS Master——控制中心：

- 为节点提供命名和注册服务



## ROS环境配置

### ROS版本

| Distribution    | Release date    | Ubuntu |
| --------------- | --------------- | ------ |
| Melodic Morenia | May 23rd, 2018  | 18.04  |
| Kinetic Kame    | May 23rd, 2016  | 16.04  |
| Indigo Igloo    | July 22nd, 2014 | 14.04  |



### 安装

[kinetic]: http://wiki.ros.org/kinetic/Installation/Ubuntu
[melodic]:http://wiki.ros.org/melodic/Installation/Ubuntu
[noetic]:http://wiki.ros.org/noetic/Installation/Ubuntu

#### 1.2 Setup your sources.list

Setup your computer to accept software from packages.ros.org.

```shell
$sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#官网这个太慢，使用中科大源
$sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
#清华镜像源
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### 1.3 Set up your keys

```shell
$sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

#### 1.4 Installation

```shell
$sudo apt-get update
##opt1
$sudo apt-get install ros-kinetic-desktop-full
##opt2 ROS, rqt, rviz, and robot-generic libraries
$sudo apt-get install ros-melodic-desktop-full
##opt3 ROS package, build, and communication libraries. No GUI tools.
$sudo apt install ros-melodic-ros-base
```

如果已经安装好ROS，但cmake卸载了，那么很多ROS包也会被卸载，需要从**1.4**重新执行安装过程。

#### 1.5 Environment setup

```shell
# 环境变量设置（全局有效），推荐设置
# for kinetic ubuntu16.04
$echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
# for melodic ubuntu18.04
$echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$source ~/.bashrc

# 仅本终端有效，不推荐
source /opt/ros/kinetic/setup.bash
```

#### 1.6 Dependencies for building packages

```bash
$sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
#或者
$sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

```

**Initialize rosdep**

```shell
#如果没有安装rosdep则运行
$sudo apt install python-rosdep
#初始化 rosdep
$sudo rosdep init
$rosdep update

# update执行结果如下
alex@alex-desktop:~$ rosdep update
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
Skip end-of-life distro "bouncy"
Skip end-of-life distro "crystal"
Add distro "dashing"
Add distro "eloquent"
Add distro "foxy"
Skip end-of-life distro "groovy"
Skip end-of-life distro "hydro"
Skip end-of-life distro "indigo"
Skip end-of-life distro "jade"
Add distro "kinetic"
Skip end-of-life distro "lunar"
Add distro "melodic"
Add distro "noetic"
Add distro "rolling"
updated cache in /home/alex/.ros/rosdep/sources.cache
```

#### catkin工具包

安装好ROS后，你会发现`catkin_make`可以运行但`catkin`相关命令无法运行，这是因为需要再单独安装下[catkin tools](https://catkin-tools.readthedocs.io/en/latest/index.html#)，安装命令如下：

```
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```



#### Note

**Desktop-Full Install: (Recommended)** : ROS, [rqt](http://wiki.ros.org/rqt), [rviz](http://wiki.ros.org/rviz), robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception

#### python3

由于neotic(ubuntu2020)之前版本的ROS1是默认不支持python3的，故我们需要添加python3的支持。很简单，例如如果你安装的是`melodic`，只需要运行：

```shell
$ pip3 install catkin-tools
$ pip3 install rospkg
```

在Python脚本最上面添加解释器说明：



### 安装路径

所有的package都在目录如：`/opt/ros/kinetic`下。



### 问题及解决

#### rosdep无法初始化

**问题描述：**在运行`$sudo rosdep init`时会弹出如下错误：

`ERROR: cannot download default sources list from:
https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
Website may be down.`

**解决方法一：**

由于raw.githubusercontent.com地址被封了，无法进行域名解析故访问，可以查询其真实IP并添加至hosts文件。

1. 查询真实IP

通过IPAddress.com首页,输入raw.githubusercontent.com查询到真实IP地址

IPAddress.com网址：https://www.ipaddress.com/

我们可以看到其IP为：

- [185.199.108.133](https://www.ipaddress.com/ipv4/185.199.108.133)
- [185.199.109.133](https://www.ipaddress.com/ipv4/185.199.109.133)
- [185.199.110.133](https://www.ipaddress.com/ipv4/185.199.110.133)
- [185.199.111.133](https://www.ipaddress.com/ipv4/185.199.111.133)

2. 添加至hosts文件

打开终端，输入`sudo vi /etc/hosts`，添加如下：

```
199.232.96.133  raw.githubusercontent.com
```

然后重新执行`sudo rosdep init`即可。

**解决方法二：**

参考：https://blog.csdn.net/qq_25368751/article/details/104248464

第一步：

由于无法更新下载*20-default.list*文件，那么直接新建好了。

直接在/etc目录下新建/ros/rosdep/sources.list.d/20-default.list文件（注意sudo rosdep init失败时，/etc下并没有/ros目录，需要依次逐级新建），然后将https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list内容粘贴进去，内容如下：

```
# os-specific listings first
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx
# generic
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte
# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
```

第二步：

在/usr/lib/python2.7/dist-packages/rosdep2/sources_list.py中顶部直接插入两行代码取消SSL验证。

```
import ssl
ssl._create_default_https_context = ssl._create_unverified_context
```

再次rosdep update。

#### cmake卸载导致ros无法运行

根据安装过程，从**1.4**重新开始安装。

#### roscore无法运行

错误原因是python连接到了python3.x上了，要想正确使用ros, python应该连接到python2.7上，解决步骤如下。

第一步：执行：ls -n /usr/bin/python

发现python软链接到了python3.7

第二步删除软链接：sudo rm -rf /usr/bin/python

第三步：然后再重新建立到python2.7的软链接：

sudo ln -s /usr/bin/python2.7 /usr/bin/python

第四步：问题解决。

## ROS使用基础

### ROS命令

ROS提供了一系列的命令供用户使用，而且可以很方便的使用`Tab`进行补全，而且可以通过两次`Tab`列出可选择的命令提示。

| roc cmd   | cmd format  | func          |
| --------- | ----------- | ------------- |
| rqt_graph | $ rqt_graph | 绘制ROS运行图 |
|           |             |               |
|           |             |               |



| ros cmd    | cmd format                                | func                                                    |
| ---------- | ----------------------------------------- | ------------------------------------------------------- |
| rosrun     | $ rosrun *pkg_name*  *node_name*          | 启动ROS节点                                             |
| rosnode    | $ rosnode list                            | 查看当前运行的节点                                      |
|            | $ rosnode info *node_name*                | 查看节点具体信息(Publications, Subscriptions, Services) |
|            | $ rosnode ping *node_name*                |                                                         |
| rostopic   | $ rostopic list                           | 查看当前话题列表                                        |
|            | $ rostopic pub                            | 发布                                                    |
|            | $ rostopic type                           | 查看topic的类型信息                                     |
|            | $ rostopic echo                           | 查看话题内容                                            |
| rosmsg     | $ rosmsg show *msg_name*                  | 查看msg具体信息                                         |
| rosservice | $ rosservice list                         | 查看当前service列表                                     |
|            | $ rosservice call *service_name*  *param* | 调用service                                             |
| rosparam   | $ rosparam list                           | 查看参数列表                                            |
|            | $ rosparam get *param_name*               | 获取参数                                                |
|            | $ rosparam set *param_name* *value*       | 设置参数                                                |
|            | $ rosparam dump *yaml_file*               |                                                         |
|            | $ rosparam load *yaml_file*               |                                                         |
| rospack    | $rospack list                             | 查看当前所有package及其路径                             |
|            |                                           |                                                         |





### 工作空间Workspace

catkin编译系统下的工作空间结构

```
workspace_folder/          -- WORKSPACE
|--src/                    -- SOURCE SPACE
  |--CMakeLists.txt        -- The 'toplevel' CMake file
  |--package_1/
    |--CMakeLists.txt
    |--package.xml
    |--include/
    |--src/
    ...
  |--package_n/
    |--CMakeLists.txt
    |--package.xml
    ...
|--build/                   -- BUILD SPACE
    CATKIN_IGNORE
|--devel/                   -- DEVELOPMENT SPACE(set by CMAKE_INSTALL_PREFIX)
  |--bin/
  |--etc/
  |--include/
  |--lib/                   -- 各个package的编译结果
  |--share/
  |--.catkin
  |--env.bash
  |--setup.bash
  |--setup.sh
  ...
|--install/                -- INSTALL SPACE(set by CMAKE_INSTALL_PREFIX)
  |--bin/
  |--etc/
  |--include/
  |--lib/
  |--share/
  |--.catkin
  |--env.bash
  |--setup.bash
  |--setup.sh
  ...
```

#### 工作空间的创建与编译

创建工作空间

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```

编译工作空间

```
$ cd ~/catkin_ws/
$ catkin_make
```

Note:`catkin_make`会创建`build`和`devel`文件夹，如果需要创建INSTALL SPACE(install文件夹)，使用命令`catkin_make install`创建

#### 环境变量

设置环境变量

```
$source devel/setup.bash
```

检查环境变量

```
echo $ROS_PACKAGE_PATH
```

注意，如果不希望每次都运行添加环境变量的命令，可以将命令添加至文件`~/.bashrc`中，打开该文件在最后添加：

```
source /home/alex/catkin_ws/devel/setup.bash
```

Note:

- 按**Ctrl+H**可以显示文件夹的隐藏文件。
- .bashrc文件更改后，需要重启终端才能生效。
- you must put all **source** before **export**!



### 功能包package

安装ROS功能包

使用`apt-get install`即可安装，并将其中的package安装指ROS目录下，例如`/opt/ros/melodic/share/turtle_tf`。

```shell
sudo apt-get install ros-melodic-ros-tutorials
```



创建功能包

命令为`$catkin_create_pkg <package_name> [depend1] [depend2]`

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg test_pkg std_msgs rospy roscpp
```

编译功能包

```
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash #添加环境变量
```

Note: 只有功能包的路径添加至环境变量，功能包才能够被使用

#### package.xml

- 存放package作者信息

- 存放许可类型(MIT, GPL, etc)

- 功能包运行编译的依赖信息

#### CMakeLists.txt

cmake编译规则

在cmake的**build**部分定义了每个节点编译后的名字(即rosrun命令中的节点名字)，一般来说都将.cpp文件名作为节点的名字。

```cmake
add_executable(velocity_publisher src/velocity_publisher.cpp)
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})
```



### Build with python

1. python file should be start with "#!/usr/bin/env python"
2. you must change the file with "chmod 777 xxx.py"

## 话题topic

### 话题基本使用示例

我们将实现如下图所示的话题通信，首先在工作空间下新建一个package命名为**learning_topic**，也即在/catkin_ws/src下创建了一个learning_topic文件夹。

<img src="https://gitee.com/bpnotes/pic-museum/raw/master/pictures/topic_example.PNG" alt="topic_example.PNG" style="zoom:80%;" />

#### 定义话题消息

我们将以两个节点进行话题通信为例，说明自定义msg的步骤，这里话题通信的msg是关于Person的信息。

1、在package文件夹的**msg**文件夹中定义**Person.msg**文件

```c
string name
uint8 sex
uint8 age

uint8 unknown=0
uint8 male=1
uint8 female=0
```

2、在package文件夹的**package.xml**文件中添加功能包依赖

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

3、在package文件夹的**CMakeLists.txt**添加编译选项

```cmake
find_package(
message_generation
)

add_message_files(
FILES
Person.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
```

4、进行catkin_make编译生成语言相关文件

编译完成后，可在工作空间**devel/include**文件夹下找到与package同名的文件夹，并且有**Person.h**文件。

5、使用

```c
#include <ros/ros.h>
#include "learning_topic/Person.h"
```



#### Publisher实现

在package文件夹中新建一个person_publisher.cpp文件，在头部包含头文件`#include pkg_name/Person.h`，注意pkg_name即learning_topic。

1、实例化一个publisher

```c
// 消息类型为pkg_name::Person,其中pkg_name是package的文件夹名字
// 话题名为/person_info
// 队列长度为10
ros::Publisher person_info_pub = 
               n.advertise<pkg_name::Person>("/person_info", 10);
```

2、实例化一个msg

```c
pkg_name::Person person_msg;
person_msg.name = "Tom";
person_msg.age = 18;
person_msg.sex = pkg_name::Person::male;
```

3、发布消息

```c
person_info_pub.publish(person_msg)
```

4、配置编译规则

在CMakeLists.txt添加如下

```cmake
add_executable(person_publisher src/person_publisher.cpp)
target_link_libraries(person_publisher ${catkin_LIBRARIES})
add_dependencies(person_publisher ${PROJECT_NAME}_generate_messages_cpp)
```

其中**person_publisher**即为生成的节点名字（可执行程序）。

#### Subscriber实现

在package文件夹中新建一个person_subscriber.cpp文件，在头部包含头文件`#include pkg_name/Person.h`，注意pkg_name即learning_topic。

1、定义回调处理函数

```c
void callback(const pkg_name::Person::ConstPtr& msg)
{
	ROS_INFO("Person Info: name %s age %d sex %d", msg->name.c_str(),
	                                               msg->age, msg->sex);
}
```

2、实例化subscriber，注册回调函数

```c
ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, callback);

# make sure ros::spin() run! Don't have while loop before spin()
ros::spin();
```

注意：

1）ros::spin()是ros的死循环，用于更新ros内部的运行状态，如果ros不进行更新将收不到订阅的消息；

2）如果是希望在用户的while循环中更新ros状态，可以使用ros::spinOnce()函数！

3、配置编译规则

在CMakeLists.txt添加如下

```cmake
add_executable(person_subscriber src/person_subscriber.cpp)
target_link_libraries(person_subscriber ${catkin_LIBRARIES})
add_dependencies(person_subscriber ${PROJECT_NAME}_generate_messages_cpp)
```

其中**person_subscriber**即为生成的节点名字（可执行程序）。

#### 测试

```shell
$roscore
$rosrun pkg_name person_subscriber
$rosrun pkg_name person_publisher
```

### 传递附加参数

same callback multiple subscription

@param callback_args: additional arguments to pass to the
 callback. This is useful when you wish to reuse the same callback for multiple subscriptions



https://answers.ros.org/question/232204/passing-multiple-arguments-to-a-callback-c/

https://www.cnblogs.com/TIANHUAHUA/p/8418818.html



https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/msg.py

TypeError("publish() can be called with arguments or keywords, but not both.")

## 服务service

service是服务器端(server)和客户端(client)之间进行通讯的消息机制，在通信过程中由client发起request，server收到请求消息后进行处理并返回response。

### 服务基本使用示例

我们将实现如下图所示的服务通信，首先在工作空间下新建一个package命名为**learning_service**，也即在/catkin_ws/src下创建了一个learning_service文件夹。

<img src="https://gitee.com/bpnotes/pic-museum/raw/master/pictures/service_example.PNG" style="zoom:80%;" />

#### 自定义服务数据

1、定义srv文件

在package文件夹的**srv**文件夹中新建**Person.srv**文件，并添加如下：

```c
string name
uint8 sex
uint8 age

uint8 unknown=0
uint8 male=1
uint8 female=0
--- 
bool success
string result
```

在srv文件中，使用`---`用于分割request和response，Person.srv文件就规定了一个消息的数据结构。

注意：

返回值共两个，不能再定义其他变量了。

2、在package.xml中添加功能包依赖

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

3、在CMakeLists.txt添加编译选项

```cmake
find_package(
message_generation
)

add_service_files(
FILES
Person.srv
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
message_runtime
)
```

4、进行catkin_make编译生成语言相关文件

编译完成后，可在工作空间**devel/include**文件夹下找到与package同名的文件夹，并且有*Person.h, PersonRequest.h,PersonResponse.h*文件。

#### 客户端client

在package文件夹中新建一个person_client.cpp文件，在头部包含头文件`#include pkg_name/Person.h`。

1、实例化一个ServiceClient

```c
// 其中/show_person是service名子
ros::service::waitForService("/show_person")
//pkg_name::Person为服务数据类型
ros::ServiceClient person_client = n.serviceClient<pkg_name::Person>("/show_person")
```

2、初始化服务请求数据

```c
//Person就是文件Person.srv的文件名！
pkg_name::Person srv;
srv.request.name = "Tom";
srv.request.age = 18;
srv.request.sex = pkg_name::Person::Request::male;
```

3、访问服务

```c
person_client.call(srv);
```

**call**方法是一个阻塞函数，发送请求后一直等待响应结果。

4、服务响应结果

```c
ROS_INFO("Show person result: %s", srv.reponse.result.c_str());
```

5、配置编译规则

在CMakeLists.txt添加如下

```cmake
add_executable(person_client src/person_client.cpp)
target_link_libraries(person_client ${catkin_LIBRARIES})
add_dependencies(person_client ${PROJECT_NAME}_gencpp)
```

其中**person_server**即为生成的节点名字（可执行程序）。

#### 服务端server

在package文件夹中新建一个person_server.cpp文件，在头部包含头文件`#include pkg_name/Person.h`。

1、定义回调处理函数

```c
bool callback(pkg_name::Person::Request &req, pkg_name::Person::Response &res)
{
    ROS_INFO("Person: name %s age %d sex %d", req.name.c_str(), req.age, req.sex);
    res.result = "OK";
    res.success = true;
    return true;
}
```

2、创建server，注册回调函数

```c
ros::ServiceServer person_server = n.advertiseService("/show_person", callback);
```

3、配置编译规则

在CMakeLists.txt添加如下

```cmake
add_executable(person_server src/person_server.cpp)
target_link_libraries(person_server ${catkin_LIBRARIES})
add_dependencies(person_server ${PROJECT_NAME}_gencpp)
```

其中**person_server**即为生成的节点名字（可执行程序）。

## 节点图

运行命令

```shell
$ rosrun rqt_graph rqt_graph
```



```shell
$ pip install rospkg
```





## rostest

rostest工具是ROS提供的测试工具，该工具基于gtest框架，其实质是roslaunch工具的功能扩展，允许跨越多节点进行测试。rostest测试时文件格式可以是.test或.launch，且文件内包含<test>标签。

如果使用catkin_make时运行test，命令格式如下：

```
#
catkin_make run_tests_<node_name>
```

## ros自带的常用测试

```shell
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```



## rosbag

rosbag 主要用于记录、回放、分析 rostopic 中的数据

常用指令

- `record`：用指定的话题录制一个 bag 包
- `info`：显示一个 bag 包的基本信息，比如包含哪些话题
- `play`：回放一个或者多个 bag 包
- `check`：检查一个 bag 包在当前的系统中是否可以回放和迁移
- `compress`：压缩一个或多个 bag 包
- `decompress`：解压缩一个或多个 bag 包
- `reindex`：重新索引一个或多个损坏 bag 包

### rosbag record

订阅活动的topic发布者并记录，默认会生成一个以时间命名的`year-moth-day-hour-min-sec.bag`的文件。



- 记录所有topic

```shell
$ rosbag record -a
```

- 记录感兴趣topic

```shell
$ rosbag record /topic_name1 /topic_name2
```

- 指定生成数据包的名字

```shell
$ rosbag record -O filename.bag /topic_name1
```

- 在launch文件中使用

```shel
<node pkg="rosbag" type="record" name="rosbag_record" args="/topic1 topic2"/>
```



### rosbag play

- 在回放过程中按**空格暂停**，常见用法如下，回放单个 bag：

```text
rosbag play record.bag
```

- 回放多个 bag，基于全局时间间隔播放：

```text
rosbag play record1.bag record2.bag
```

- 开始播放立刻暂停，按空格继续：

```text
rosbag play --pause record.bag
```

- 以录制的一半频率回放：

```text
rosbag play -r 0.5 --pause record.bag
```

- 指定回放频率，默认 100HZ：

```text
rosbag play --clock --hz=200 record.bag
```

- 循环播放：

```text
rosbag play -l record.bag
```

### rosbag info

```shell
$ rosbag info filename.bag
```





## 实战例程

### 小海龟Turtle

### 一个小海龟

#### 启动ROS Master

```bash
$roscore
```

#### 启动节点

1、启动仿真界面节点：turtlesim_node

```bash
$rosrun turtlesim turtlesim_node
```

2、启动控制节点：turtle_teleop_key

```bash
$rosrun turtlesim turtle_teleop_key
```

#### 控制指令

```bash
#小海龟转圈圈，设置线速度和角速度，r设置每秒发送次数
$rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0"
```



### 两个小海龟

ref:https://blog.csdn.net/weixin_44747240/article/details/104803046

创建小海龟

```bash
$rosservice call /spawn "x: 3.0
y: 5.0
theta: 0.0
name: 'hello'"
```

控制特定小海龟

```bash
$rostopic pub -r 10 /hello/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0"
```



## 生成安装包

在ROS中，有两种路径可以打包：1）将ROS包上传至github，利用ROS自动生成到构建仓库（buildform），2）在本地打包为debian。

在本地打包时，可以通过使用ROS提供的**bloom-generate**进行打包，其安装命令如下：

```
sudo apt-get install python-bloom fakeroot
```

### 打包自定义的msg

我们将首先进行对msg打包演示：

1. 首先准备一个ros工作空间，建立一个package（alex_msg），添加msg文件夹并添加几个.msg文件，同时对package下的`package.xml`和`CMakeLists.txt`进行相应修改加入对新添加的msg文件的编译，最后运行编译。
2. 打开terminal切换至`alex_msg`包路径，运行如下：

```
roscd test_msgs
bloom-generate rosdebian --os-name ubuntu --ros-distro melodic
```





## VSCode

### 智能提示

选择设置->Command Palette，输入configuration，选择c/c++ Edit configuration，打开c_cpp_properties.json后，添加ros的头文件路径即可！

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "/opt/ros/melodic/include",
                "/usr/include",
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc",
            "cStandard": "gnu17",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-arm"
        }
    ],
    "version": 4
}
```



## 问题和解决

### python文件无法运行

```shell
allex@alex-Mi:~/Desktop/flocking_uestc$ rosrun flocking drone_mavros.py
[rosrun] Couldn't find executable named drone_mavros.py below /home/allex/Desktop/flocking_uestc/src/flocking
[rosrun] Found the following, but they're either not files,
[rosrun] or not executable:
[rosrun]   /home/allex/Desktop/flocking_uestc/src/flocking/scripts/drone_mavros.py
```

右键该文件，选择属性，进入权限，然后勾选“可作为可执行程序”即可。





