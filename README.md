注：这是一个实时记录摄像头数据的ROS功能包

1.安装
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone https://github.com/shangjie-li/image-saver.git
    cd ..
    catkin_make

2.运行
第一步：
    source devel/setup.bash
    roslaunch image_saver demo.launch
第二步：
    source devel/setup.bash
    rosrun image_saver save_image_trigger trigger_rate startIndex
