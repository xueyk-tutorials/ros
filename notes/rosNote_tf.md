# 命令使用

- rosrun tf tf_echo [reference_frame] [target_frame]

​    获取两个坐标系的tf信息并打印

- rosrun tf tf_monitor [reference_frame] [target_frame]
- rosrun rqt_tf_tree rqt_tf_tree

​    显示当前tf树；

- rosrun tf view_frames

​    获取当前的tf树并保存至PDF文件中；

## 查看两个坐标系的相对关系

可以通过`rosrun tf tf_echo <frame1> <frame2>`查看两个坐标系的相对关系。这里是frame1作为根坐标系，求frame2在frame1下的偏移和旋转。

以小海龟为例：

```shell
# 输出以turtle1为参考，turtle2的坐标和旋转
# T2_to_1 = Tw_to_1 * T2_to_w
$ rosrun tf tf_echo turtle1 turtle2
At time 1648021095.121
- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.557, 0.831]
            in RPY (radian) [0.000, 0.000, -1.181]
            in RPY (degree) [0.000, 0.000, -67.691]

```

# 基础

## 数据类型

http://wiki.ros.org/tf/Overview/Data%20Types

tf中基本的数据类型如下：

| **Type**   | **tf**           | 备注                                                         |
| ---------- | ---------------- | ------------------------------------------------------------ |
| Quaternion | `tf::Quaternion` |                                                              |
| Vector     | `tf::Vector3`    |                                                              |
| Point      | `tf::Point`      |                                                              |
| Pose       | `tf::Pose`       |                                                              |
| Transform  | `tf::Transform`  | https://docs.ros.org/en/api/tf/html/c++/classtf_1_1Transform.html |



```c++
nav_msgs::Odometry odom;
tf::Quaternion q;
tf::quaternionMsgToTF(odom.pose.pose.orientation, q);

tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

//欧拉角转四元素
tf::createQuaternionMsgFromRollPitchYaw(double r, double p, double y);
tf::createQuaternionMsgFromYaw(double y);
```








```shell
rosrun rqt_tf_tree rqt_tf_tree
```



## 发布tf

```c++
#include <tf/transform_broadcaster.h>
// 创建tf广播器
static tf::TransformBroadcaster br;

// 实例化Transform坐标变换
tf::Transform transform;
tf::Quaternion q;
q.setRPY(0, 0, M_PI/4);
transform.setRotation(q2);
transform.setOrigin(tf::Vector3(10.0, 20.0, 0.0));

// 广播坐标变换至ROS后台tf服务器
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_earth", "base_body"));


```

## 接收tf

### 接收最新的

```c++
tf::TransformListener listener;
tf::StampedTransform transform;

// 获取最新时刻，base_body到base_earth的变换
listener.lookupTransform("base_earth", "base_body", ros::Time(0), transform2);
// tf::Vector3 = transform2.getOrigin()
cout << "x= " << transform2.getOrigin().x() 
     << ", y= " << transform2.getOrigin().y() 
    << ", z= " << transform2.getOrigin().z() << endl;
// tf::Quaternion = transform2.getRotation()
cout << "x= " << transform2.getRotation().x() 
     << " y=" << transform2.getRotation().y() 
    << " z=" << transform2.getRotation().z() 
    << " w=" << transform2.getRotation().w() << endl;
```

### 接收特定时刻的

```c++
tf::TransformListener listener;
tf::StampedTransform transform;

ros::Time     tt = ros::Time::now();   // 获取当前时刻
ros::Duration du = ros::Duration(1.0);

cout << tt << ", sec=" << tt.sec << ", nsec=" << tt.nsec << endl;
tt = ros::Time::now() - ros::Duration(1.0); // 获取1秒之前
cout << tt << ", sec=" << tt.sec << ", nsec=" << tt.nsec << endl;
cout << du << ", sec=" << tt.toSec() << endl;

listener.waitForTransform("/turtle2", "/turtle1", tt, ros::Duration(3.0));
listener.lookupTransform("/turtle2", "/turtle1", tt, transform);
```



