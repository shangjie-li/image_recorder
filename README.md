# Image-saver

注：这是一个实时记录摄像头数据的ROS功能包。

# 安装
 - 建立工作空间并拷贝这个库：
   ```Shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/shangjie-li/image-saver.git
   cd ..
   catkin_make
   ```

# 运行
 - 运行`image_saver`：
   ```Shell
   source devel/setup.bash
   roslaunch image_saver demo.launch
   ```
 - 运行`save_image_trigger`：
   ```Shell
   source devel/setup.bash
   rosrun image_saver save_image_trigger trigger_rate startIndex
   ```

# Installation
 - Create a workspace and clone this repository:
    ```Shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/shangjie-li/image-saver.git
   cd ..
   catkin_make
   ```

# Run
 - Run `image_saver`:
   ```Shell
   source devel/setup.bash
   roslaunch image_saver demo.launch
   ```
 - Run `save_image_trigger`:
   ```Shell
   source devel/setup.bash
   rosrun image_saver save_image_trigger trigger_rate startIndex
   ```   
   
   
