# HIKROBOT-MVS-CAMERA-ROS
海康威视工业相机sdk的ros驱动包，支持配置参数,照片已经转码为rgb格式。

# Install
```
mkdir -p ~/ws_hikrobot_camera/src
git clone https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS.git ~/ws_hikrobot_camera/src/hikrobot_camera
cd ~/ws_hikrobot_camera
catkin_make
```
# launch启动
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera.launch
```
# launch启动
用 rviz 订阅 /hikrobot/rgb 查看照片
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera_rviz.launch
```
