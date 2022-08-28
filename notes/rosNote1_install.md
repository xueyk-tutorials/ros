# ROS笔记——安装

## ROS版本

| Distribution    | Release date    | Ubuntu |
| --------------- | --------------- | ------ |
| Melodic Morenia | May 23rd, 2018  | 18.04  |
| Kinetic Kame    | May 23rd, 2016  | 16.04  |
| Indigo Igloo    | July 22nd, 2014 | 14.04  |

## 参考

[kinetic]: http://wiki.ros.org/kinetic/Installation/Ubuntu
[melodic]:http://wiki.ros.org/melodic/Installation/Ubuntu
[noetic]:http://wiki.ros.org/noetic/Installation/Ubuntu

## ubuntu源配置

请先配置ubuntu国内镜像源

## 安装

**Desktop-Full: (Recommended)** : ROS, [rqt](http://wiki.ros.org/rqt), [rviz](http://wiki.ros.org/rviz), robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception



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

## python3支持

由于neotic(ubuntu2020)之前版本的ROS1是默认不支持python3的，故我们需要添加python3的支持。很简单，例如如果你安装的是`melodic`，只需要运行：

```shell
$ pip3 install catkin-tools
$ pip3 install rospkg
```

在Python脚本最上面添加解释器说明：

## 安装路径

所有的package都在目录如：`/opt/ros/kinetic`下。

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



## 问题及解决

### rosdep无法初始化

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

### cmake卸载导致ros无法运行

根据安装过程，从**1.4**重新开始安装。

### roscore无法运行

错误原因是python连接到了python3.x上了，要想正确使用ros, python应该连接到python2.7上，解决步骤如下。

第一步：执行：ls -n /usr/bin/python

发现python软链接到了python3.7

第二步删除软链接：sudo rm -rf /usr/bin/python

第三步：然后再重新建立到python2.7的软链接：

sudo ln -s /usr/bin/python2.7 /usr/bin/python

第四步：问题解决。





