# Detect Object
Gazebo 中基于PX4无人机对特定物体进行识别并撞击或穿越（例如气球、门框）的仿真程序。 

The simulation programme based on Gazebo and PX4 to fly across or crash object.

## 效果展现
<img width="1920" alt="result" src="https://github.com/Lovely-XPP/Detect_Balloon/assets/66028151/386dda38-4502-4a04-9297-63c2003000fe">

<img width="1920" alt="result" src="https://github.com/Lovely-XPP/Detect_Object/assets/66028151/eea777ad-4298-4943-95e2-80a2e15fcfae">

## 依赖
ROS 包：`px4_cmd`

## 编译
```bash
mkdir ~/catkin_ws && cd ~/catkin_ws && \
mkdir src && cd src && \
git clone https://github.com/Lovely-XPP/Detect_Balloon.git && \
cd .. && catkin_make
```

## 运行
### 撞击气球
```bash
source ~/catkin_ws/setup.bash
bash $(rospack find dectect_balloon)/sh/sim_balloon.sh
```

### 穿越门框
```bash
source ~/catkin_ws/setup.bash
bash $(rospack find dectect_balloon)/sh/sim_doorframe.sh
```
