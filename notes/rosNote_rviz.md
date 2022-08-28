rviz是为了解决数据可视化，例如机器人模型的可视化、图像数据的可视化、地图数据的可视化等。

## 坐标系显示

红色为X轴

绿色为Y轴

蓝色为Z轴



## 参数文件

如果要可视化显示的话题非常多，如果每次打开rviz都去手动一个个重新添加是非常不方便的。幸好，rviz提供了配置文件保存机制，可以将当前窗口打开的话题保存为`.rviz`配置文件，后续希望再次开启，可以有两种办法：

1. 先打开rviz，然后通过File->Open Config，打开保存的配置文件。

2. 通过launch启动，将配置文件作为参数传给rviz节点：

   ```shell
   <launch>
     <node name="rviz_alex" pkg="rviz" type="rviz" args="-d $(find your_pkg_name)/launch/alex.rviz" required="true" />
   </launch>
   ```

   

### RobotModel

机器人的模型是从参数服务器中读取并加载，也就是说其他节点启动时必须在参数中加载一个模型（urdf）至参数服务器中（一般都是通过launch文件加载）。

