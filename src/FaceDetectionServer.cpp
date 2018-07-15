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
    // Register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&FaceDetectionAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FaceDetectionAction::preemptCB, this));

    // Subscribe to the data topic of interest
    // Some example topics: /kinect2/qhd/image_color , /camera/image_raw /pepper_robot/camera/front/image_raw
    image_sub_ = nh_.subscribe<sensor_msgs::Image>( "/camera/image_raw", 1, &FaceDetectionAction::analysisCB, this);

    
    image_transport::ImageTransport it(nh_);
    // Publish the stream for the live demo on pepper
    detector_pub = it.advertise("detector_stream", 1000);
    
    as_.start();
  }

  ~FaceDetectionAction(void)
  {
  }


  static void read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, vector<string>& names, char separator = ';')
  {
    // Read image path, id and name from the csv file
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
    
    // Detect faces    
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(im_width, im_height) );

    // if(faces.size() == 0)
    //   {
    // 	// Stop if no face is found
    // 	return false;
    //   }
    cv::waitKey(10);
    // Recognize faces
    for( int i = 0; i < faces.size(); i++ )
      {
	// Process face by face:
	Rect face_i = faces[i];
	// Crop the face from the image
	Mat face = frame_gray(face_i);
	Mat face_resized;
	cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
	int prediction = 0;
	double predicted_confidence = 0.0;
	// Find out id
	model->predict(face_resized, prediction, predicted_confidence);
	ROS_INFO_STREAM("Prediction: " << prediction << ", confidence: " << predicted_confidence);
	if(prediction > -1)
	  {
	    // If found, draw arectangle around the found with the name
	    rectangle(original, face_i, CV_RGB(0, 255,0), 1);

	    std::stringstream var;
	    var << "Prediction = " << names[prediction];
	    string box_text = var.str();
	    int pos_x = std::max(face_i.tl().x - 10, 0);
	    int pos_y = std::max(face_i.tl().y - 10, 0);
	    putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
	    result_.face_id = prediction;
	    result_.face_name = names[prediction];
	    ROS_INFO_STREAM("Erkenne: " << names[prediction]);
	    goal_ = 1;
	  }
	else
	  {
	    // Go to learn action because the face is unknown
	    ROS_INFO("Face unknown");
	    goal_ = -1;
	  }
	// cv::waitKey(500);
	    
      }
    
    // Publish and show current frame
    detector_pub .publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", original).toImageMsg());
    cv::imshow( "Capture - Face detection", original );
    return true;
  }

  static void trainModel(std::vector<cv::Mat>& images, std::vector<int>& labels, cv::Ptr<cv::face::EigenFaceRecognizer>& model)
  {
    // Train the model with the loaded images and the id's
    model->train(images, labels);
  }

  void goalCB()
  {
    // Accept the new goal
    goal_ = as_.acceptNewGoal()->detection_mode;

    // Load external files: face-classifier and the csv-file
    face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml");
    fn_csv = string("/home/hanna/action_ws/src/face_learning_actions/src/TrainData.csv");
  }

  void preemptCB()
  {
    // Set the action state to preempted if no face on the frame is found
    as_.setPreempted();
  }

  void analysisCB(const sensor_msgs::Image::ConstPtr& msg)
  {
    // Make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    cv::waitKey(200);

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
    std::vector<cv::Mat> images;
    std::vector<int> labels;
    std::vector<string> names;
    int num_components = 30;
    double threshold = 7000.0;
    // Then if you want to have a cv::FaceRecognizer with a confidence threshold,
    // create the concrete implementation with the appropiate parameters:
    Ptr<cv::face::EigenFaceRecognizer> model = EigenFaceRecognizer::create(num_components, threshold);

    // Read csv and load images, labels and names
    try
      {
	read_csv(fn_csv, images, labels, names);
      }
    catch (cv::Exception& e)
      {
	ROS_INFO_STREAM("Error openening file " << fn_csv << ". Reason: " << e.msg);
	// Nothing more we can do
	exit(1);
      }

    // Set size of images
    int im_width = images[0].cols;
    int im_height = images[0].rows;
    // Train the model with the images and id's
    trainModel(images, labels, model);
    cv::CascadeClassifier haar_cascade;

    // Run the detection.
    // If a face is found, set the action to succeeded.
    // If a face is found but unknown set action to aborted.
    // If no face is found set the acton to preemted. The action will be called again to wait for faces.
    if(detectAndDisplay( cv_ptr->image, im_width, im_height, model, names))
      {
	if(goal_ == -1)
	  {
	    as_.setAborted();
	  }
	else
	  {
	    ROS_INFO_STREAM("name: " << result_.face_name);
	    as_.setSucceeded(result_);
	  }
      }
    else
      {
	as_.setPreempted();
      }
	
  }

protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<face_learning_actions::FaceDetectionAction> as_;
  std::string action_name_;

  face_learning_actions::FaceDetectionFeedback feedback_;
  face_learning_actions::FaceDetectionResult result_;

  // Subscriber and Publisher
  ros::Subscriber image_sub_;
  image_transport::Publisher detector_pub ;
  
  //  Global variables
  cv::CascadeClassifier face_cascade;
  std::string fn_csv;
  cv::VideoCapture capture;
  cv::Mat frame;
  int goal_;
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FaceDetectionActionServer");

  FaceDetectionAction fd(ros::this_node::getName());
  ros::spin();

  return 0;
}
