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


class ImageSaver
{
public:
	ImageSaver();
	~ImageSaver();
	bool init();
	
private:
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
	void saveImageCallback(const std_msgs::Int32::ConstPtr &msg);
	
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	ros::Subscriber sub_image_;
	ros::Subscriber sub_save_trigger_;
	
	std::string image_topic_;
	std::string image_path_;
	bool use_trigger_;
	cv_bridge::CvImageConstPtr imgPtr_;
};

ImageSaver::ImageSaver():
	imgPtr_(nullptr),
	nh_private_("~")
{
}

ImageSaver::~ImageSaver()
{

}


bool ImageSaver::init()
{
	image_topic_ = nh_private_.param<std::string>("image_topic", "/image_raw");
	image_path_ =  nh_private_.param<std::string>("image_path","");
	use_trigger_ = nh_private_.param<bool>("use_trigger",false);
	
	sub_image_ = nh_.subscribe(image_topic_, 1, &ImageSaver::imageCallback, this);
	
	if(image_path_[image_path_.length()-1] != '/')
		image_path_ = image_path_ + "/";
	
	if(use_trigger_)
		sub_save_trigger_ = nh_.subscribe("/save_image_trigger", 1, &ImageSaver::saveImageCallback, this);

	return true;
}

void ImageSaver::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	imgPtr_ = cv_bridge::toCvShare(msg, "bgr8");
	
	if(!use_trigger_)
	{
		
		static std_msgs::Int32 _MSG; _MSG.data =1;
		static std_msgs::Int32::ConstPtr msgPtr= boost::shared_ptr<const std_msgs::Int32>(&_MSG);
		
		_MSG.data ++;
		
		saveImageCallback(msgPtr);
	}
}

void ImageSaver::saveImageCallback(const std_msgs::Int32::ConstPtr &msg)
{ 
	
	if(imgPtr_ != nullptr && !imgPtr_->image.empty())
	{
		std::string file_name = image_path_+ std::to_string(msg->data) + ".jpg";
		bool ok = cv::imwrite(file_name, imgPtr_->image);
		
		if(ok)
			std::cout << "save image in " << file_name << std::endl;
		else
			ROS_ERROR("%s is not exist!",image_path_.c_str());
			
		imgPtr_ = nullptr;
	}
		
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "image_saver",ros::init_options::AnonymousName);
	ImageSaver imageSaver;
	if(imageSaver.init())
		ros::spin();
		
	return 0;
}

