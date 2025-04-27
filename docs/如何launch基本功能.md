# 如何launch基本功能

## 〇、前提条件

在工作空间目录下编译：

```bash
catkin_make
source devel/setup.sh
```

安装`explore_lite`包：

```bash
sudo apt install ros-noetic-explore-lite
```

此后，**每在工作空间目录下打开一个新的终端，都一定记得：**

```bash
source devel/setup.sh
```

* * *

## 一、仿真场景

### 1.`launch`文件的功能

* [`gmapping_sim.launch`](..\src\gazebosim_demo\launch\gmapping_sim.launch)用于在仿真环境中实现建图功能

* [`library_sim.launch`](..\src\gazebosim_demo\launch\library_sim.launch)用于在仿真环境中实现导航、物品抓取的完整功能

### 2.在仿真场景中建图

运行以下命令，打开仿真场景：

```bash
roslaunch gazebosim_demo gmapping_sim.launch
```

#### (1)手动建图

##### a) 在工作空间目录下，打开一个新的终端，并输入如下命令：

```bash
source devel/setup.sh
rosrun gazebosim_demo vel_ctrl_node.py
```

##### b) 控制机器人在环境中运动

在这个终端中**单击**`w` `s` `a` `d` `shift`或空格键，机器人会如下表所示运动。机器人默认认为仿真环境中的**门**是在自己**背后**的。

|单击键位|机器人运动|
|:---:|:---:|
|`w`|前进|
|`s`|后退|
|`d`|向右|
|`a`|向左|
|`shift`或`space`|停在原地|

##### c) 保存地图

当rviz中的地图建立完毕后，可以通过如下命令保存地图：

```bash
rosrun map_server map_saver -f map
```

地图文件（一个`.yaml`和一个`.pgm`文件）将会保存在终端所在的目录下。

#### (2)自动建图

**在工作空间目录下打开一个新的终端，运行如下命令**：

```bash
source ./src/realbot_demo/scripts/auto_mapping.sh
```

等待rviz中建图基本完成后，可在终端所在目录找到地图文件（一个`.yaml`和一个`.pgm`文件）。

### 3.在仿真场景中操控机器人

运行以下命令，打开仿真场景：

```bash
roslaunch gazebosim_demo library_sim.launch
```

#### (1) 自动避障的自主导航

##### a) 在工作空间目录下打开两个终端，分别运行：

终端一：

```bash
source devel/setup.sh
rosrun navigation simple_goal.py
```

终端二：

```bash
source devel/setup.sh
rosrun navigation target.py
```

##### b) 给机器人提供导航目的地

在运行`target.py`的终端(上述的终端二)中，输入`0`并且回车，然后按照提示输入想要去的坐标即可。

> 如果想要抓取书籍，那么可让机器人前往`x=3.77, y=2`的方位。

##### c) 开始导航

可以在机器人运动路径前方动态放置障碍物，测试自动避障效果。

##### d) 导航结束后再次导航

在运行`target.py`的终端(上述的终端二)中，输入`0`并且回车，再次输入想要去的坐标即可。

#### (2) 抓取物品

##### a) 让机器人运动到`x=3.77, y=2`的方位

##### b) 在工作空间目录下打开一个新的终端，运行如下命令

```bash
rosrun realbot_demo obj_grab_node.py
```

##### c) 待该终端显示`done`后，抓取完毕

注意：如果运行`obj_grab_node.py`后终端没有黄色字体提示，或者黄色字体停止出现达5s，则可以在`ctrl+c`后再次运行上述命令，重新尝试抓取。

#### (3) 搬运物品

在抓取物品后启动导航模块进行导航即可。

#### (4) 放置物品

在工作空间目录下打开一个新的终端，运行如下命令

```bash
rosrun realbot_demo obj_place_node.py
```

* * *

## 三、常见问题

### 1. 启动一次gazebo后，后面再次启动总是失败？

第一次启动gazebo后，`gzserver`的进程还残留着，杀死即可。

查看`gzserver`或`gzclient`的进程号：

```bash
ps -ef
```

杀死那些进程：

```bash
kill <进程号>
```

### 2. 启动`launch`文件后怎么退出？

我没有设计退出的按钮，因此直接在终端`ctrl+c`强制退出即可。

### 3. 自动建图提示没有`explore`包？

```bash
sudo apt install ros-noetic-explore-lite # 请注意不是下划线_，而是减号-
```

### 4. 导航总是失败？

机器人认为要导航到的方位没法容纳机器人。要么因为目的地不是地面，要么因为空间太狭小，机器人没法站立。

> 备注：在`src\navigation\launch`下的两个`yaml`文件中的`robot_radius`规定了机器人要站立至少需要的圆的半径大小。
