#include <ros/ros.h>
#include "opencv2/objdetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <actionlib/server/simple_action_server.h>
#include <face_learning_actions/FaceDetectionAction.h>
#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/aruco.hpp"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace cv::face;
using namespace std;

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
    // /kinect2/qhd/image_color , /camera/image_raw /pepper_robot/camera/front/image_raw
    image_sub_ = nh_.subscribe<sensor_msgs::Image>( "/camera/image_raw", 1, &FaceDetectionAction::analysisCB, this);
    image_transport::ImageTransport it(nh_);
    detector_pub = it.advertise("detector_stream", 1000);
    
    as_.start();
  }

  ~FaceDetectionAction(void)
  {
  }


  static void read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, vector<string>& names, char separator = ';')
  {
    ifstream file(filename.c_str(), std::ifstream::in);
    if (!file)
      {
	ROS_INFO_STREAM("No file " << file << "from: " << filename);
	std::string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
      }
    std::string line, path, classlabel, name;
    while (getline(file, line))
      {
	if(!line.empty())
	  {
	    std::stringstream liness(line);
	    getline(liness, path, separator);
	    getline(liness, classlabel, separator);
	    getline(liness, name);
	    if(!path.empty() && !classlabel.empty())
	      {
		images.push_back(imread(path, 0));
		labels.push_back(atoi(classlabel.c_str()));
		names.push_back(name.c_str());
	      }
	  }
      } 
  }
  
  bool detectAndDisplay( cv::Mat frame, int im_width, int im_height, Ptr<cv::face::EigenFaceRecognizer> model, vector<string> names)
  {
    std::vector<cv::Rect> faces;
    cv::Mat frame_gray;
    Mat original = frame.clone();
    
    cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );
    
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(80, 80) );

    if(faces.size() == 0)
      {
	ROS_INFO("Kein Gesicht gefunden.");
	return false;
      }
    cv::waitKey(10);
    for( int i = 0; i < faces.size(); i++ )
      {
	// Process face by face:
	Rect face_i = faces[i];
	// Crop the face from the image. So simple with OpenCV C++: !!!
	Mat face = frame_gray(face_i);
	Mat face_resized;
	cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
	int prediction = 0;
	double predicted_confidence = 0.0;
	model->predict(face_resized, prediction, predicted_confidence);
	ROS_INFO_STREAM("Prediction: " << prediction << ", confidence: " << predicted_confidence);
	if(prediction > -1)
	  {
	    rectangle(original, face_i, CV_RGB(0, 255,0), 1);
	    string box_text = format("Prediction = %d", prediction);
	    int pos_x = std::max(face_i.tl().x - 10, 0);
	    int pos_y = std::max(face_i.tl().y - 10, 0);
	    putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
	    result_.face_id = prediction;
	    result_.face_name = names[prediction];
	    goal_ = 1;
	  }
	else
	  {
	    ROS_INFO("No face found");
	    goal_ = -1;
	  }

	    
      }
    
    detector_pub .publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", original).toImageMsg());
    cv::imshow( "Capture - Face detection", original );
    return true;
  }

  static void trainModel(std::vector<cv::Mat>& images, std::vector<int>& labels, cv::Ptr<cv::face::EigenFaceRecognizer>& model)
  {
    model->train(images, labels);
  }

  void goalCB()
  {
    ROS_INFO_STREAM(action_name_ << " executing goalCB()");
    
    // reset helper variables
    number_of_faces_ = 0;

    // accept the new goal
    goal_ = as_.acceptNewGoal()->detection_mode;


    if(face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"))
    
      fn_csv = string("/home/hanna/action_ws/src/face_learning_actions/src/TrainData.csv");

    image_database_path = "/home/hanna/action_ws/src/face_learning_actions/src/images/";
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

    cv::waitKey(700);

    goal_ = 0;
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
    // find faces; if found face known result = 1
    std::vector<cv::Mat> images;
    std::vector<int> labels;
    std::vector<string> names;
    
    int num_components = 1;
    double threshold = 6000.0;
    // Then if you want to have a cv::FaceRecognizer with a confidence threshold,
    // create the concrete implementation with the appropiate parameters:
    Ptr<cv::face::EigenFaceRecognizer> model = EigenFaceRecognizer::create(num_components, threshold);
    // Ptr<cv::face::FisherFaceRecognizer> model = FaceRecognizer::create(num_components, threshold);

    try
      {
	read_csv(fn_csv, images, labels, names);
      }
    catch (cv::Exception& e)
      {
	ROS_INFO_STREAM("Error openening file " << fn_csv << ". Reason: " << e.msg);
	// nothing more we can do
	exit(1);
      }
    
    int im_width = images[0].cols;
    int im_height = images[0].rows;
    trainModel(images, labels, model);
    cv::CascadeClassifier haar_cascade;

    haar_cascade.load("/usr/share/opencv/haarcascades/haacrcascade_frontalface_default.xml");

    if(detectAndDisplay( cv_ptr->image, im_width, im_height, model, names))
      {
	if(goal_ == -1)
	  {
	    
	    ROS_INFO("No success.");
	    as_.setAborted();
	  }
	else
	  {
	    ROS_INFO("Successed.");
	    ROS_INFO_STREAM("id: " <<  result_.face_id);
	    ROS_INFO_STREAM("name: " << result_.face_name);
	    as_.setSucceeded(result_);
	  }
      }
    else
      {
	ROS_INFO("No face found");
	as_.setPreempted();
      }
	
  }

protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<face_learning_actions::FaceDetectionAction> as_;
  std::string action_name_;

  face_learning_actions::FaceDetectionFeedback feedback_;
  face_learning_actions::FaceDetectionResult result_;
  ros::Subscriber image_sub_;
  image_transport::Publisher detector_pub ;

  int goal_;
  bool success_;
  int number_of_faces_;

  std::string image_database_path;

  /** Global variables */
  cv::CascadeClassifier face_cascade;
  std::string fn_csv;
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
