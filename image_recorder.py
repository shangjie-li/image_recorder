import matplotlib.pyplot as plt # For WARNING: QApplication was not created in the main() thread.

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

try:
    import cv2
except:
    import sys
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2

import os
import sys
import threading
import time
import numpy as np
import argparse


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Image Recorder')
    parser.add_argument('--sub_image1', default='/cam_rgb/usb_cam/image_raw', type=str,
                        help='The name of ROS topic.')
    parser.add_argument('--sub_image2', default='/cam_t/usb_cam/image_raw', type=str,
                        help='The name of ROS topic.')
    parser.add_argument('--frame_rate', default=1, type=int,
                        help='The frame rate to record.')

    global args
    args = parser.parse_args(argv)


image1_lock = threading.Lock()
image2_lock = threading.Lock()


def get_time_stamp():
    cur_time = time.time()
    local_time = time.localtime(cur_time)
    data_head = time.strftime("%Y-%m-%d %H:%M:%S", local_time)
    data_secs = (cur_time - int(cur_time)) * 1000
    time_stamp = "%s.%03d" % (data_head, data_secs)
    stamp = "".join(time_stamp.split()[0].split("-")) + "".join(time_stamp.split()[1].split(":"))
    return stamp.replace('.', '')


def image1_callback(image):
    global image1_frame
    image1_lock.acquire()
    image1_frame = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    image1_lock.release()


def image2_callback(image):
    global image2_frame
    image2_lock.acquire()
    image2_frame = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    image2_lock.release()


def timer_callback(event):
    global image1_frame
    image1_lock.acquire()
    cur_frame1 = image1_frame.copy()
    image1_lock.release()
    
    global image2_frame
    image2_lock.acquire()
    cur_frame2 = image2_frame.copy()
    image2_lock.release()
    
    global path_rgb, path_t
    time_stamp = get_time_stamp()
    path1 = path_rgb + '/rgb_' + time_stamp + '.jpg'
    path2 = path_t + '/t_' + time_stamp + '.jpg'
    print('Saving image to %s.' % path1)
    print('Saving image to %s.' % path2)
    cv2.imwrite(path1, cur_frame1[:, :, ::-1]) # to BGR
    cv2.imwrite(path2, cur_frame2[:, :, ::-1]) # to BGR


if __name__ == "__main__":
    parse_args()
    rospy.init_node('image_recorder', anonymous=True, disable_signals=True)
    
    path = './img' + get_time_stamp()
    os.makedirs(path)
    path_rgb = path + '/img_rgb'
    os.makedirs(path_rgb)
    path_t = path + '/img_t'
    os.makedirs(path_t)
    
    image1_frame, image2_frame = None, None
    rospy.Subscriber(args.sub_image1, Image, image1_callback, queue_size=1)
    rospy.Subscriber(args.sub_image2, Image, image2_callback, queue_size=1)
    while image1_frame is None or image2_frame is None:
        time.sleep(0.1)
        print('Waiting for topic %s and %s...' % (args.sub_image1, args.sub_image2))
    print('  Done.\n')
    
    rospy.Timer(rospy.Duration(1.0 / args.frame_rate), timer_callback)
    rospy.spin()
