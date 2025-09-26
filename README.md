# 遥操重定向系统

## 系统介绍

本系统由两部分组成，以基于linkerhand核心包和重定向程序两部分组成，其中核心包主要收集了跟遥操手套的连接办法，协议处理，数据算法解析等

重定向程序是基于ROS1和ROS2开发的，由一个节点和两个publisher的话题组成

手套标定软件通过UDP协议进行数据推送到重定向程序

本程序是基于Python3.8开发的，请准备好基于此系统的环境(后续更新到pypi)

### 下载地址

1. 重定向系统程序地址:
   ROS1:[点此跳转](https://github.com/linkerbotai/linker_telop_sdk/tree/main/linkertelopsdk/ros1/install)
   ROS2:[点此跳转](https://github.com/linkerbotai/linker_telop_sdk/tree/main/linkertelopsdk/ros2/install)
2. linkerhand核心whl离线包地址
   Linux_x64平台：[点此跳转](https://github.com/linkerbotai/linker_telop_sdk/blob/main/linkertelopsdk/whl/linkerhand-2.6.3-cp38-cp38-linux_x86_64.whl)
   Linux_arm64平台：[点此跳转](https://github.com/linkerbotai/linker_telop_sdk/blob/main/linkertelopsdk/whl/linkerhand-2.6.3-cp38-cp38-linux_aarch64.whl)

### 准备工作

我司在GITHUB开源了摇操重定向系统的SDK，可通过该程序连接手套设备，目前支持的设备有

1、LINKERFORCE手套

2、VTRDYN数据手套

3、UDEXREAL手套

4、NOVA数据手套

其他厂家可咨询我司进行定制化开发

可通过如下链接进行下载

```
git clone https://github.com/linker-bot/linkerhand-ros-teleop.git
cd linkertelopsdk
```

### 安装核心包

核心包目前只开放了Ubuntu平台下的Arm64平台和x64平台

Arm平台的参考对象如：树莓派，Jetson NX，Rk3688等

x64平台的的参考对象一般都是台式机的Intel和AMD
输入以下命令，根据指示条完成即可成功安装，途中有些依赖包需要联网更新，注意保持联网状态。

由于部分版本存在旧版本没有删掉关键性文件的问题，因此需要事先卸载已安装的包，如果未安装包则无需理会

```
## 以下仅在安装过的包需要执行
pip uninstall linkerhand
```

```
pip install linkerhand-2.6.8-cp38-cp38-linux_aarch64.whl
```

### 安装遥操重定向程序

在该目录下分别执行以下内容（ROS1版）

```
catkin_make install
```

在该目录下分别执行以下内容（ROS2版）

```
colcon build
```

### 运行遥操重定向程序

在该目录下分别执行以下内容（ROS1版）

```
source ./install/setup.bash 
rosrun ros_linkerhand_retarget handretarget.py
```

在该目录下分别执行以下内容（ROS2版）

```
source ./install/setup.bash
ros2 run linkerhand_retarget handretarget 
```

运行途中可能存在因为包的版本不对导致报错的可能，尤其是scipy，树莓派默认安装的是1.4.x版本，而我们需要更新到1.10.1版本，则执行以下指令，则可以把系统中的scipy卸载并安装到用户系统中

```
sudo pip uninstall scipy
pip install scipy==1.10.1``
```

使用过程中可能会存在权限不足的问题导致无法运行，使用以下代码

```
chmod 777 -R *
```

### 程序配置

该目录输入ls命令后应该可以看到install目录，程序配置相关的内容在该目录下的相对路径（ROS1版）

```
cd install/lib/ros_linkerhand_retarget/config/
ls -1
```

该目录输入ls命令后应该可以看到install目录，程序配置相关的内容在该目录下的相对路径（ROS2版）

```
cd install/linkerhand_retarget/share/linkerhand_retarget/config/
ls -1
```

输入以上命令后正常应该进入config的目录，ls -1后即可看到以下清单

```
base_config.yml
body_custom_pose.yml
body_unity_pose.yml
hand_config.yml
human_hand_info.yml
linker_hand_info.yml
model_config.yml
retarget_config.yml
speed_config.yml
```

其中我们要唯一修改的配置文件是base\_config.yml，其他配置文件均不可修改

以下是针对base\_config.yml的配置内容展开进行介绍，

1. UDP相关配置

   配置UDP地址和端口，根据Studio软件的设置做相应的配置。

   ```
   udp:
     ip: 192.168.11.130
     port: 7000
     serverport: 5551
   ```

   如：mHandStudio.exe的UDP配置的是192.168.11.200，端口7000，相应的内容要替换成对应的地址和端口，同时System的useudp选项要保持为true
2. 配置机械手类型

   ```
     robotname_r: l10
     robotname_l: l10
   ```

* LinkerHand\_L10系列的手要配置成:l10
* LinkerHand\_L20系列的手要配置成:l20
* LinkerHand\_O7系列的手要配置成:o7
* LinkerHand\_L7系列的手要配置成:l7
* LinkerHand\_T25系列的手要配置成:t25
* LinkerHand\_L25系列的手要配置成:l25
* LinkerHand\_L16系列的手要配置成:l16

3. 配置数据打印

   ```
   debug:
     joint_pub_debug: false
     joint_motor_debug_r: true
     joint_motor_debug_l: false
   ```

* joint\_pub\_debug:设置为true可打印Topic Pub后的内容，刷新率较慢，可左右手同时显示
* joint\_motor\_debug\_r:设置为true可快速刷新右手的输出数据，范围在255-0，代表张开到聚拢的变化幅度
* joint\_motor\_debug\_l:设置为true可快速刷新左手的输出数据，范围在255-0，代表张开到聚拢的变化幅度

4. 配置数据源

   ```
     motion_type: udexreal
     motion_device: eric
   ```

* motion\_type:使用UDEXREALl宇叠手套要设置为udexreal
* motion\_type:使用LINKERFORCE力反馈手套要设置为linkerforce
* motion\_type:使用VTRDYN动捕手套要设置为vtrdyn
* motion\_device:仅在UDEXREALl下可以激活使用，需要输入软件所对应的角色，用于绑定数据源
