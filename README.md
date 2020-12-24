# image_recorder

ROS package for recording images

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/image_recorder.git
   cd ..
   catkin_make
   ```

## 参数配置
 - 修改`image_recorder/launch/image_recorder.launch`
   ```Shell
   <param name ="sub_topic" value="/image_rectified"/>
   <param name ="save_path" value="$(find image_recorder)/images"/>
   <param name ="frame_interval" value="5"/>
   <param name ="start_index" value="1"/>
   ```
    - `sub_topic`指明订阅的图像话题。
    - `save_path`指明保存图像的路径，第一次使用时需手动创建。
    - `frame_interval`为保存图像帧的间隔，应根据图像发布的频率进行设置。
    - `start_index`为保存图像名的起始索引，在每次运行时应按需修改，否则会覆盖先前保存的图像。

## 运行
 - 启动`image_recorder`
   ```Shell
   roslaunch image_recorder image_recorder.launch
   ```

