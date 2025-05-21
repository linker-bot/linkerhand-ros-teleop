# 遥操重定向应用-SenseGlove Nova2版

## 本系统介绍

本系统是基于SenseGlove Nova2力反馈手套开发的遥操重定向系统

本系统分为由SenseCom应用程序，UdpServerSDK，UdpClientSDK，LinkerHand组件四个部分组成

1. SenseCom应用程序运行在Windows平台，Linux暂时没有做适配开发，该程序由原厂提供，据了解该程序只能运行在x64的平台下，Arm平台涉及到蓝牙串口驱动的兼容性问题，建议有条件的开发者可以访问官网进一步了解。
   该应用程序负责通过蓝牙串口的方式物理连接SenseGlove Nova2力反馈手套
2. UdpServerSDK编译后应用程序名称为linkersgn，该程序是基于Visual C++平台开发的，官方暂时没有提供基于MingW的编译套件，但是提供了Linux的so库，有条件的开发者可以参考官方的SDK进行开发，我方后续有时间会进行补充性的更新，添加基于Linux的应用程序，同理，Linux应用程序需要捆绑Linux版的SenseCom才可以使用
   该程序以SDK形式公开，符合Apachje-2.0协议许可规范

   该应用程序负责通过API方式获取SenseCom进行数据的交互，并通过UDP的方式广播出去
3. UdpClientSDK名称为handretarget-nova，该程序属于Python应用，可应用在多个平台，以SDK形式公开，符合Apachje-2.0协议许可规范

   该应用程序负责通过获取来自UDP传输的内容，与LinkerHand组件相结合，驱动对应的LinkerHand系列机械手，目前支持市面上所有的LinkerHand全系列机械手
4. LinkerHand组件为运行handretarget-nova所必须的Python组件，提供了以下几个平台
   Linux_x64
   Linux_arrch64
   Windows_x86

##
