#include <ros/ros.h>
#include "opencv2/objdetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <actionlib/server/simple_action_server.h>
#include <face_learning_actions/FaceDetectionAction.h>

class FaceDetectionAction
{
public:
    
  FaceDetectionAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&FaceDetectionAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FaceDetectionAction::preemptCB, this));

    //subscribe to the data topic of interest
    image_sub_ = nh_.subscribe<sensor_msgs::Image>( "/camera/image_raw", 1, &FaceDetectionAction::analysisCB, this);

    as_.start();
  }

  ~FaceDetectionAction(void)
  {
  }

  int detectAndDisplay( cv::Mat frame)
  {
    ROS_INFO_STREAM("Start detect and Display function");
    std::vector<cv::Rect> faces;
    cv::Mat frame_gray;
    
    cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );
    
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(80, 80) );
    // ROS_INFO_STREAM("Start work with detected faces count: " << faces.size());
    cv::waitKey(100);
    for( size_t i = 0; i < faces.size(); i++ )
      {
	cv::Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );

	cv::rectangle(frame, cv::Point( faces[i].x + faces[i].width + 20, faces[i].y + faces[i].height + 20),
		      cv::Point(faces[i].x - faces[i].width/2 + 10, faces[i].y - faces[i].height/2), cv::Scalar(255,0, 0));
	
	cv::Mat faceROI = frame_gray( faces[i] );
      }
    cv::imshow( "Capture - Face detection", frame );
    return faces.size();
  }


  void goalCB()
  {
    ROS_INFO_STREAM(action_name_ << " executing goalCB()");
    
    // reset helper variables
    number_of_faces_ = 0;

    // accept the new goal
    goal_ = as_.acceptNewGoal()->detect_face;

    ROS_INFO_STREAM(action_name_ << " accepted goal: " << goal_);

    face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml");
    
    eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml"); 
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void analysisCB(const sensor_msgs::Image::ConstPtr& msg)
  {

    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    // ROS_INFO("analysis cb");

    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }
  number_of_faces_ = detectAndDisplay( cv_ptr->image);

  // if(number_of_faces_ >= 1)
  //   {
  //     ROS_INFO_STREAM("finished with: " << number_of_faces_);
  //     as_.setSucceeded(result_);
  //   }
    feedback_.number_of_faces=number_of_faces_;
    as_.publishFeedback(feedback_); 
  }

protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<face_learning_actions::FaceDetectionAction> as_;
  std::string action_name_;

  face_learning_actions::FaceDetectionFeedback feedback_;
  face_learning_actions::FaceDetectionResult result_;
  ros::Subscriber image_sub_;

  bool goal_;
  bool success_;
  int number_of_faces_;

  /** Global variables */
  std::string face_cascade_name;
  std::string eyes_cascade_name;
  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;
  cv::VideoCapture capture;
  cv::Mat frame;
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FaceDetectionActionServer");

  FaceDetectionAction fd(ros::this_node::getName());
  ros::spin();

  return 0;
}
