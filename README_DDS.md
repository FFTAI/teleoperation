# 1. DDS 使用说明

本项目包含 DDS 控制、适配[FFTAI遥操](https://github.com/FFTAI/teleoperation)的相关部分，以下是docker内启动及测试流程。

---

## 文件说明

- `bin/fftai_dds_bridge`: 运行 DDS Bridge 的可执行文件，需要加载BRIDGE_CONFIG_FILE配置文件
- `libraries`: 与 DDS 相关的动态库文件
- `src/arm_core_tele`: 基于DDS编写的机器人控制文件
  - `test.py`: 测试脚本，用于验证机器人控制功能。其主要功能包括：  
    - 通过 DDS 通信实现对机械臂的控制 
    - 用于调试和验证机器人控制模块是否工作正常 
    - 函数接口说明：
        - np.deg2rad（get_all_current_pos_deg）适配grx-client下获取的32个电机弧度制数据
        - DDSPipeline.move_joints(control_positions)中接收的是[(joint_name, position_deg)]
        - 基本顺序为 get_pvc_states() --> pose_solver_.solve() --> pose_solver_.inverse()


---

## 使用步骤

### 1. 使容器访问宿主机的 X11 显示

```bash
xhost +local:root
```

### 2. 构建镜像

```bash
git clone https://github.com/Geo-JTao/teleoperation.git
mv teleoperation teleoperation_dds && cd teleoperation_dds
git checkout feature/teleop_dds 
mv Dockerfile ../Dockerfile && cd ../ 
docker build -t <your image name> .
# 查看镜像ID
docker images
```
#### 3. 创建容器
```bash
sudo docker run --gpus all -it --net=host -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics --privileged=true -v /dev/shm:/dev/shm --hostname root --name <your container name > <your image id > /bin/bash
# 启动容器并进入
docker start <your container name > 
docker exec -it <your container name >  /bin/bash
```
#### 4. 终端测试
```bash
# 终端1 启动Bridge
./bridge.sh 
# 终端2 测试控制（左臂将被抬起90度）
cd src/
python arm_core_tele/test.py 
```

# 2. 遥操接口适配
## 基础使用说明
参考 [FFTAI遥操](https://github.com/FFTAI/teleoperation)

## 新增文件说明
- `configs/teleop_gr1_12dof.yaml` main函数加载的配置文件，可在此处更换不同类型的机器人、灵巧手、相机的配置文件

- `configs/robot/gr1_dds.yaml` 基于dds控制的机器人配置文件
- `configs/hand/fourier_12dof_dexpilot_dds.yaml` 12自由度灵巧手的配置文件
- `configs/camera/realsense.yaml`realsense相机的配置文件，注意修改top对应的相机编号，可通过realsense-viewer连接后的info获取

## Adapter适配说明
- `src/teleoperation/adapter/hands/fourier_dexhand.py ` 12自由度灵巧手
- `src/teleoperation/adapter/robots/grx_dds.py ` 基于dds控制的机器人

## 容器内使用
```bash
# 终端1 启动Bridge
./bridge.sh 
# 终端2 开启遥操
cd src/
./teleop.sh 
```