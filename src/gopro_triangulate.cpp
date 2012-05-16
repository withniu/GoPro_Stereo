#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/CvBridge.h>
#include "std_msgs/String.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "GoPro_Stereo/pos2d.h"
#include "GoPro_Stereo/pos3d.h"


class ImageTri {
private:
	ros::NodeHandle nh_;
//	image_transport::Publisher pub_left_;		// Publish images
//	image_transport::Publisher pub_right_;		// Publish images
//	image_transport::Publisher pub_disp_;
	
	ros::Publisher pub_gopro_,pub_ladybug_;	

//	ros::Subscriber sub_left_;			// Subscribe filenames
//	ros::Subscriber sub_right_;
	ros::Subscriber sub_gopro_;
	ros::Subscriber sub_ladybug_;

//	cv::Mat image_left_;
//	cv::Mat image_right_;
//	sensor_msgs::ImagePtr img_msg_left_;
//	sensor_msgs::ImagePtr img_msg_right_;	

	GoPro_Stereo::pos3d pos3d_gopro_,pos3d_ladybug_;	// Customized msg
	
	cv::Mat P_left_,P_right_;
	cv::Mat Q_;						// Q matrix for reprojection to 3D

//	cv::Mat pos_gopro_, pos_ladybug_;			// Position estimation
//	float distance_gopro_, distance_ladybug_;		// Distance estimation
//	float bearing_gopro_, bearing_ladybug_;			// Bearing estimation
	
	
public:
	ImageTri();
	virtual ~ImageTri();


	void callback_gopro(const GoPro_Stereo::pos2d& msg);
	void callback_ladybug(const GoPro_Stereo::pos2d& msg);
//	void callback_left(const std_msgs::String::ConstPtr& msg);
//	void callback_right(const std_msgs::String::ConstPtr& msg);


	void Run();
	void triangulate(float, float, float, float);
	void initStereo();
};

ImageTri::ImageTri() {
//	image_transport::ImageTransport it(nh_);;
//	pub_left_ = it.advertise("gopro/image_rover_left", 1);
//	pub_right_ = it.advertise("gopro/image_rover_right", 1);
//	pub_disp_ = it.advertise("gopro/image_disp", 1);
	pub_gopro_ = nh_.advertise<GoPro_Stereo::pos3d>("pos3d_gopro",1);
	pub_ladybug_ = nh_.advertise<GoPro_Stereo::pos3d>("pos3d_ladybug",1);
	sub_gopro_ = nh_.subscribe("gopro/position2d", 1, &ImageTri::callback_gopro, this);	
	sub_ladybug_ = nh_.subscribe("ladybug/position2d", 1, &ImageTri::callback_ladybug, this);	
		
	initStereo();

}

ImageTri::~ImageTri() {
}

void ImageTri::callback_gopro(const GoPro_Stereo::pos2d& msg) {
	if (msg.flag) {
		ROS_INFO("GoPro keypoints received.");
		triangulate(msg.x1,msg.y1,msg.x2,msg.y2);
		pub_gopro_.publish(pos3d_gopro_);
	} 
}
void ImageTri::callback_ladybug(const GoPro_Stereo::pos2d& msg) {
	if (msg.flag) {
		ROS_INFO("Ladybug keypoints received.");
		pos3d_ladybug_.bearing = 0;
		pub_ladybug_.publish(pos3d_ladybug_);
	} 
}

void ImageTri::triangulate(float x1, float y1, float x2, float y2) {
	cv::Mat pts1 = (cv::Mat_<double>(3,1) << x1,y1,1.0);
	cv::Mat pts2 = (cv::Mat_<double>(3,1) << x2,y2,1.0);
	cv::Mat pts4d;
	CvMat p_left = P_left_;
	CvMat p_right = P_right_;
	CvMat pts_left = pts1;
	CvMat pts_right = pts2;
	CvMat pts_4d = pts4d;
	
	cv::Keypoint keypoint_left(x1,y1);
	cv::Keypoint keypoint_right(x2,y2);
	
	cv::vector<cv::Keypoint> keypoints_left;
	cv::vector<cv::Keypoint> keypoints_right;
	
	keypoints_left.pushback(keypoint_left);
	keypoints_right.pushback(keypoint_right);


	cvTriangulatePoints(&p_left, &p_right, &pts_left, &pts_right, &pts_4d);
	
	std::cout << pts4d << std::endl;

	pos3d_gopro_.X = pts4d.at<double>(1,1);
	pos3d_gopro_.Y = pts4d.at<double>(2,1);
	pos3d_gopro_.Z = pts4d.at<double>(3,1);	
	pos3d_gopro_.flag = true;
	pos3d_gopro_.distance = sqrt(pos3d_gopro_.X * pos3d_gopro_.X + pos3d_gopro_.Y * pos3d_gopro_.Y + pos3d_gopro_.Z * pos3d_gopro_.Z);
	pos3d_gopro_.bearing = 0;

}

void ImageTri::initStereo() {
	
	cv::Mat K_left = (cv::Mat_<double>(3,3) << 1956.51116, 0, 2088.45305, 0, 1988.78594, 1465.49329, 0, 0, 1);
	cv::Mat K_right = (cv::Mat_<double>(3,3) << 1968.00644, 0, 1781.81829, 0, 2008.64322, 1428.07669, 0, 0, 1);
	cv::Mat distCoeffs_left = (cv::Mat_<double>(5,1) <<   -0.30801,0.07204,-0.00134,-0.00716,0.00000);	
	cv::Mat distCoeffs_right = (cv::Mat_<double>(5,1) <<  -0.28965,0.06474,0.00153,0.00951,0.00000);
	cv::Size2i imageSize(3840,2880);
	cv::Mat R = (cv::Mat_<double>(3,3) <<   0.9838,0.0147,0.1788,-0.0140,0.9999,-0.0049,-0.1788,0.0023,0.9839);
	cv::Mat T = (cv::Mat_<double>(3,1) << -302.55382,3.25888,29.16032);
	cv::Mat R_left,R_right;
	
	cv::stereoRectify(K_left,distCoeffs_left,K_right,distCoeffs_right,imageSize,R,T,R_left,R_right,P_left_,P_right_,Q_);

//	std::cout << R_left << std::endl;
//	std::cout << R_right << std::endl;
	std::cout << "P_left" << P_left_ << std::endl;
	std::cout << "P_right" << P_right_ << std::endl;
//	std::cout << Q_ << std::endl;

}

void ImageTri::Run() {
	while( ros::ok() ){
		ros::spin();
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "stereo_triangulation");
	ImageTri ImageTri;
	ImageTri.Run();
}
