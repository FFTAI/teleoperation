# introduction

此文件夹包含了机器人模型文件以及一些配置文件，以下将介绍如何构建一个新的机器人并构建所需要的配置文件。

我们以**gr2t2**举例：

meshes: 包含了机器人模型的stl文件

robot_config: 包含了一个名为"hardware.json"的文件，后续将详细介绍。其他的配置文件也可以放入此文件夹中。例如 “hardware_dds_config.json” 包含了dds的一些配置。

robot.urdf：模型文件。

在代码编译时，会将整个文件夹安装在系统目录，程序在启动时可以传入其路径初始化fourier_hardware以及fourier_hardware_dds . 可以参照已经提供的一些配置文件构建新的机器人的配置文件。



## hardware.json

此文件是最重要的文件，控制程序根据此文件构建 motor，control_group, hardware，接下来将详细介绍本文件中的一些配置项，请不要在不清楚配置项具体意义的情况下修改此文件。

**robot_name** ：机器人名字

**control_step**: 控制周期，决定了一些状态估计的计算，例如hardware中状态更新线程的更新频率以及关节加速度的估计，另外还有低通滤波器的采样频率也用到了该值。

**control_groups**: 控制组，包含了机器人所有的控制组。可以按照自己的需求将机器人分成不同的控制组

**接下来介绍控制组里面的内容：**

**name**: 控制组名字，对于人形来说，可以分为 waist,head,left_manipulator,right_manipulator,left_leg,right_leg，当然也可以自定义。

**motors_kind**: 电机类型，目前仅支持fsa和fake，对于一个控制组来说，所有的电机的类型应该是一致。

**data_filter**: 是否对关节数据进行滤波，true/false ，一个控制组的所有电机应具有相同设置

**motors_mode**: 电机模式，目前支持position（位置模式），ft（力矩模式），pd（pd模式），一个控制组的电机应相同

**control_group_type**: 控制组模式，目前仅支持direct，控制组的定义在control_group.hpp中，新的控制组需要根据基类实现。

**joints**: urdf中的关节名字，对于direct 控制组而言，顺序应与motors的顺序保持一直，其他自定义控制组应注意两者顺序。

**motors**: 电机，定义了该控制组包含的电机信息。

**接下来介绍电机里面的内容：**

**type**: 电机类别，revolute/linear

**motor_name**: 对于fake 和 fsa电机，名字可以设为一个电机ip，其他电机可以自定义。

**fri_scale**：摩擦力和电机惯性力补偿系数

**ctft**: 电流转力矩系数，包含减速比，应根据具体电机型号给出

**ftc，ftv，fti**: 库伦摩擦，粘滞摩擦，转子惯量，可以由标定给出，当不涉及精细动力学控制时可以给出合理范围内的值。

**reverse_direction**: 是否对原始电机数据反向。

**filter_cf**: 低通滤波器截至频率。

