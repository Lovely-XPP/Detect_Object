# Detect_Balloon
Gazebo 中基于PX4的无人机对一个红色的气球进行识别并撞击的仿真程序。

## 效果展现
<img width="1920" alt="result" src="https://github.com/Lovely-XPP/Detect_Balloon/assets/66028151/386dda38-4502-4a04-9297-63c2003000fe">

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
```bash
source ~/catkin_ws/setup.bash
bash $(rospack find dectect_balloon)/sh/simulation.sh
```
