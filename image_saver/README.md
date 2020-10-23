# 订阅ROS图像消息，并将图片保存在本地

## Nodes

### image_saver_node
    订阅ROS图像消息，并将图片保存在本地，按照数据顺序进行命名

#### params

* **`image_topic`**[string]
    图像话题名称

* **`image_path`**[string]
    存放图片的路径

* **`use_trigger`**[bool]
    是否使用外部触发。如果不使用外部触发，将保存收到的所有图片，\
    如果使用外部触发，只有收到保存指令才会保存一帧图像，图片按照触发信号命名(见save_image_trigger)

#### subscribed
* **`/save_image_trigger`**[std_msgs::Int32]
    外部触发指令

#### run 
`roslaunch image_saver image_saver.launch `

### save_image_trigger
    定时循环发送保存图片信号

#### argvs

* **`trigger_rate`**[int]
    发布触发信号的频率

* **`startIndex`**[string]
    可选参数，默认为1。触发信号的起始编号，用于图片命名

#### Published
* **`/save_image_trigger`**[std_msgs::Int32]
    外部触发指令

#### run 
`rosrun image_saver save_image_trigger trigger_rate startIndex`
