#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <std_msgs/Int32.h>
#include <string>
#include <vector>

class ImageRecorder
{
public:
	ImageRecorder(ros::NodeHandle &nh);
	~ImageRecorder();
	
private:
	void image_cb(const sensor_msgs::Image::ConstPtr &msg);
	
private:
	std::string sub_topic_;
	std::string save_path_;
	int frame_interval_;
	int save_index_;
	
	int frame_num_;
	cv_bridge::CvImageConstPtr imgPtr_;
	
	ros::Subscriber sub_image_;
};

ImageRecorder::ImageRecorder(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/usb_cam/image_rect_color");
    nh.param<std::string>("save_path", save_path_, "");
    
    if (save_path_ == "") ROS_ERROR("save_path is not specified!");
	if (save_path_[save_path_.length() - 1] != '/') save_path_ = save_path_ + "/";
    
    nh.param<int>("frame_interval", frame_interval_, 5);
    nh.param<int>("start_index", save_index_, 1);
    
    frame_num_ = 0;
    imgPtr_ = nullptr,
	
	sub_image_ = nh.subscribe(sub_topic_, 1, &ImageRecorder::image_cb, this);
    ros::spin();
}

ImageRecorder::~ImageRecorder()
{
}

void ImageRecorder::image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
	if (frame_num_ % frame_interval_ == 0)
	{
	    imgPtr_ = cv_bridge::toCvShare(msg, "bgr8");
	    
	    if (imgPtr_ != nullptr && !imgPtr_->image.empty())
    	{
		    std::string file_name = save_path_ + std::to_string(save_index_) + ".jpg";
		
    		if (cv::imwrite(file_name, imgPtr_->image))
    			std::cout << "Save image " << file_name << std::endl;
    		else
    			ROS_ERROR("%s doesn't exist!", save_path_.c_str());
	    }
		save_index_ ++;
		imgPtr_ = nullptr;
	}
	frame_num_ ++;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_recorder");
    ros::NodeHandle nh("~");

    ImageRecorder imagerecorder(nh);
    return 0;
}

