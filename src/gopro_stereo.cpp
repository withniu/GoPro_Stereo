// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>
#include <tf/transform_broadcaster.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/gpu/gpu.hpp>        



#define DRAW 1			
#define GPU 0			// Use opencv GPU library

// Stereo Params




class GoProStereo {
// Members
private:
	int index_;		// Count for callback to switch ping-pong
	ros::Time time_;	// Timing for callback
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	ros::Publisher pub_;
   	image_transport::Publisher image_pub_;

	cv::Mat image_left_,image_right_;
		
  	typedef image_transport::SubscriberFilter ImageSubscriber;
	ImageSubscriber image_left_sub_;
	ImageSubscriber image_right_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	message_filters::Synchronizer< MySyncPolicy > sync;

// Methods
public:
	GoProStereo() :	// Constructor
			it_(nh_),
		    	image_left_sub_( it_, "/gopro/image_left", 1 ),
			image_right_sub_( it_, "/gopro/image_right", 1 ),
			sync( MySyncPolicy( 10 ), image_left_sub_, image_right_sub_ ) 
	{
		image_pub_ = it_.advertise("out", 1);		

		index_ = 0;
		time_ = ros::Time::now();
		sync.registerCallback( boost::bind( &GoProStereo::callback, this, _1, _2 ) );

#if DRAW
		cv::namedWindow("Left",CV_WINDOW_AUTOSIZE);			
		cv::namedWindow("Right",CV_WINDOW_AUTOSIZE);
//		cv::namedWindow("Left Rectified");
//		cv::namedWindow("Right Rectified");				
#endif
		
	}

	void callback(const sensor_msgs::ImageConstPtr& image_left_msg, const sensor_msgs::ImageConstPtr& image_right_msg) {
		
		// Convert image msg to cv::Mat
		cv_bridge::CvImagePtr cv_left_ptr;	
		cv_bridge::CvImagePtr cv_right_ptr;	
		try {
			cv_left_ptr = cv_bridge::toCvCopy(image_left_msg);
			cv_right_ptr = cv_bridge::toCvCopy(image_right_msg);	
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		image_left_ = cv_left_ptr->image;
		image_right_ = cv_right_ptr->image;

		image_pub_.publish(cv_left_ptr->toImageMsg());	
#if DRAW
		cv::imshow("Left", image_left_);
		cv::imshow("Right", image_right_);			
		cv::waitKey(2000);		
#endif
		ROS_INFO("Image Received.");
		std::cout << image_left_.rows << "x" << image_left_.cols << std::endl; 
		index_++;
		time_ = ros::Time::now();
	}
};




int main(int argc, char** argv) {
  ros::init( argc, argv, "GoPro_Stereo");
  GoProStereo mc;

  while( ros::ok() ){
    ros::spin();
  }

  return EXIT_SUCCESS;
}
 
