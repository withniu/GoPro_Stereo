#include "exif.h"
#include <fstream>
#include <ctime>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/CvBridge.h>
#include "std_msgs/String.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class ImagePub {
private:
	ros::NodeHandle nh_;
	image_transport::Publisher pub_left_;			// Publish left raw image
	image_transport::Publisher pub_right_;			// Publish right raw image
	image_transport::Publisher pub_rectified_left_;		// Publish left rectified image
	image_transport::Publisher pub_rectified_right_;	// Publish right rectified image
	image_transport::Publisher pub_disp_;			// Publish disparity map
	ros::Subscriber sub_left_;				// Subscribe filenames
	ros::Subscriber sub_right_;				// Subscribe filenames
	sensor_msgs::ImagePtr img_msg_left_;
	sensor_msgs::ImagePtr img_msg_right_;
	sensor_msgs::ImagePtr img_rectified_msg_left_;
	sensor_msgs::ImagePtr img_rectified_msg_right_;
	
	
	char filename_left_[20];
	char filename_right_[20];

	cv::Mat image_left_;
	cv::Mat image_right_;
	cv::Mat image_rectified_left_;
	cv::Mat image_rectified_right_;

	cv::Mat Q_;
	cv::Mat map1_left_,map2_left_;
	cv::Mat map1_right_,map2_right_;
	
	cv::StereoBM bm_;
	cv::Mat disp_;
public:
	ImagePub();
	virtual ~ImagePub();
	void callback_left(const std_msgs::String::ConstPtr& msg);
	void callback_right(const std_msgs::String::ConstPtr& msg);
	void Run();
	int time_str2int(char *time1,char *time2);
	void stereo();
	void initStereo();
};

ImagePub::ImagePub() {
	image_transport::ImageTransport it(nh_);
	pub_left_ = it.advertise("gopro/image_left", 1);
	pub_right_ = it.advertise("gopro/image_right", 1);
	pub_disp_ = it.advertise("gopro/image_disp", 1);
	pub_rectified_left_ = it.advertise("gopro/image_rectified_left", 1);
	pub_rectified_right_ = it.advertise("gopro/image_rectified_right", 1);
	sub_left_ = nh_.subscribe("filename_left", 1, &ImagePub::callback_left, this);
	sub_right_ = nh_.subscribe("filename_right", 1, &ImagePub::callback_right, this);
	initStereo();

	
}

ImagePub::~ImagePub() {
}

void ImagePub::callback_left(const std_msgs::String::ConstPtr& msg) {
	
	strcpy(filename_left_,msg->data.c_str());
	char filename[80];
	strcpy(filename,"/home/withniu/GoPro/GOPRO_L/");
	strcat(filename,msg->data.c_str());
	image_left_ = cv::imread(filename);
	// Load timestamp	
	FILE *fp = fopen(filename,"r"); 	
	Cexif exif;
	exif.DecodeExif(fp);
	fclose(fp);
	time_t rawtime;
  	time(&rawtime);
	char time_str[20];
	strcpy(time_str,ctime(&rawtime)+11);
	time_str[8] = '\0';
	ROS_INFO("%s\tSystem:%s\tEXIF:%s\tDelay:%ds",msg->data.c_str(),time_str,exif.m_exifinfo->DateTime+11,time_str2int(time_str,exif.m_exifinfo->DateTime+11));
	
	cv::WImageBuffer3_b image(cvLoadImage(filename,CV_LOAD_IMAGE_COLOR));
	img_msg_left_ = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");

	if (!strcmp(filename_left_,filename_right_)) {
		ROS_INFO("Left Filename:%s; Right Filename:%s published.",filename_left_,filename_right_);
		stereo();	
	}
}
void ImagePub::callback_right(const std_msgs::String::ConstPtr& msg) {
	
	strcpy(filename_right_,msg->data.c_str());
	char filename[80];
	strcpy(filename,"/home/withniu/GoPro/GOPRO_R/");
	strcat(filename,msg->data.c_str());
	image_right_ = cv::imread(filename);
	// Load timestamp	
	FILE *fp = fopen(filename,"r"); 	
	Cexif exif;
	exif.DecodeExif(fp);
	fclose(fp);
	time_t rawtime;
  	time(&rawtime);
	char time_str[20];
	strcpy(time_str,ctime(&rawtime)+11);
	time_str[8] = '\0';
	ROS_INFO("%s\tSystem:%s\tEXIF:%s\tDelay:%ds",msg->data.c_str(),time_str,exif.m_exifinfo->DateTime+11,time_str2int(time_str,exif.m_exifinfo->DateTime+11));
	
	cv::WImageBuffer3_b image(cvLoadImage(filename,CV_LOAD_IMAGE_COLOR));
	img_msg_right_ = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");

	if (!strcmp(filename_left_,filename_right_)) {
		ROS_INFO("Left Filename:%s; Right Filename:%s published.",filename_left_,filename_right_);
		stereo();	
	}
}

void ImagePub::stereo() {
	remap(image_left_,image_rectified_left_,map1_left_,map2_left_,cv::INTER_LINEAR);
	remap(image_right_,image_rectified_right_,map1_right_,map2_right_,cv::INTER_LINEAR);
	
	IplImage ipl_img_left = image_rectified_left_;
	IplImage ipl_img_right = image_rectified_right_;		
	img_rectified_msg_left_ = sensor_msgs::CvBridge::cvToImgMsg(&ipl_img_left, "bgr8");
	img_rectified_msg_right_ = sensor_msgs::CvBridge::cvToImgMsg(&ipl_img_right, "bgr8");
	
	ros::Time time = ros::Time::now();	
	img_rectified_msg_left_->header.stamp = time;
	img_rectified_msg_right_->header.stamp = time;

	
	pub_left_.publish(img_msg_left_);
	pub_right_.publish(img_msg_right_);	
	pub_rectified_left_.publish(img_rectified_msg_left_);
	pub_rectified_right_.publish(img_rectified_msg_right_);
	
	cv::Mat grayscale_left,grayscale_right;
	cv::cvtColor(image_rectified_left_,grayscale_left,CV_RGB2GRAY);
	cv::cvtColor(image_rectified_right_,grayscale_right,CV_RGB2GRAY);

	bm_(grayscale_left,grayscale_right,disp_);
	cv::Mat disp8;
	int numberOfDisparities  = 0;
	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((2880/8) + 15) & -16;
	bm_.state->numberOfDisparities = numberOfDisparities;
	disp_.convertTo(disp8, CV_8UC1, 255/(numberOfDisparities*16.));
	
//	cv::namedWindow("disparity", 0);
//      cv::imshow("disparity", disp8);
//      cv::waitKey(3);
	cv::Mat img_3d;
	cv::reprojectImageTo3D(disp_,img_3d,Q_);

}

void ImagePub::initStereo() {
//	cv::Mat K_left = (cv::Mat_<double>(3,3) << 1931.945, 0, 1921.791, 0, 1958.679, 1467.752, 0, 0, 1);
//	cv::Mat K_right = (cv::Mat_<double>(3,3) << 1942.080, 0, 1961.677, 0, 1972.209, 1464.904, 0, 0, 1);
//	cv::Mat distCoeffs_left = (cv::Mat_<double>(5,1) << -0.31781, 0.09116, 0.00034, -0.00394, 0.0);	
//	cv::Mat distCoeffs_right = (cv::Mat_<double>(5,1) << -0.31924, 0.08618, 0.00014, -0.00003, 0.0);
//	cv::Size2i imageSize(3840,2880);
//	cv::Mat R = (cv::Mat_<double>(3,3) <<  0.9999, 0.0107, 0.0098, -0.0106, 0.9999, -0.0126, -0.0100, 0.0125, 0.9999);
//	cv::Mat T = (cv::Mat_<double>(3,1) << -307.8305, 1.6335, 2.8248);

//	cv::Mat f_left = (cv::Mat_<double>(2,1) << 2020.33105, 2035.79394);
//	cv::Mat f_right = (cv::Mat_<double>(2,1) << 1987.09250, 2018.35899);
//	cv::Mat c_left = (cv::Mat_<double>(2,1) << 2103.36847   1494.31928);
//	cv::Mat c_right = (cv::Mat_<double>(2,1) << 1987.09250, 2018.35899);
	
	cv::Mat K_left = (cv::Mat_<double>(3,3) << 1956.51116, 0, 2088.45305, 0, 1988.78594, 1465.49329, 0, 0, 1);
	cv::Mat K_right = (cv::Mat_<double>(3,3) << 1968.00644, 0, 1781.81829, 0, 2008.64322, 1428.07669, 0, 0, 1);
	cv::Mat distCoeffs_left = (cv::Mat_<double>(5,1) <<   -0.30801,0.07204,-0.00134,-0.00716,0.00000);	
	cv::Mat distCoeffs_right = (cv::Mat_<double>(5,1) <<  -0.28965,0.06474,0.00153,0.00951,0.00000);
	cv::Size2i imageSize(3840,2880);
	cv::Mat R = (cv::Mat_<double>(3,3) <<   0.9838,0.0147,0.1788,-0.0140,0.9999,-0.0049,-0.1788,0.0023,0.9839);
	cv::Mat T = (cv::Mat_<double>(3,1) << -302.55382,3.25888,29.16032);

	
	cv::Mat R_left,R_right;
	cv::Mat P_left,P_right;


	cv::stereoRectify(K_left,distCoeffs_left,K_right,distCoeffs_right,imageSize,R,T,R_left,R_right,P_left,P_right,Q_);

	cv::initUndistortRectifyMap(K_left,distCoeffs_left,R_left,P_left,imageSize,CV_32FC1,map1_left_,map2_left_);
	cv::initUndistortRectifyMap(K_right,distCoeffs_right,R_right,P_right,imageSize,CV_32FC1,map1_right_,map2_right_);

	std::cout << R_left << std::endl;
	std::cout << R_right << std::endl;
	std::cout << P_left << std::endl;
	std::cout << P_right << std::endl;
	std::cout << Q_ << std::endl;
}

int ImagePub::time_str2int(char *time1,char *time2) {
	
	int dt = ((time1[4] - '0') * 60 + (time1[6]-'0') * 10 + time1[7] - '0' - ((time2[4] - '0') * 60 + (time2[6] - '0') * 10 + time2[7] - '0'));	// Assuming delay < 60s
	return dt < 0 ? dt + 600 : dt;
}

void ImagePub::Run() {
	while( ros::ok() ){
		ros::spin();
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
	ImagePub ImagePub;
	ImagePub.Run();
}
