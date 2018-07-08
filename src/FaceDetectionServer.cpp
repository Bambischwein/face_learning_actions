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

#include <ros/console.h>

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
    // /camera/image_raw
    image_sub_ = nh_.subscribe<sensor_msgs::Image>( "/kinect2/qhd/image_color", 1, &FaceDetectionAction::analysisCB, this);

    as_.start();
  }

  ~FaceDetectionAction(void)
  {
  }

  static void read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, char separator = ';')
  {
    ifstream file(filename.c_str(), std::ifstream::in);
    if (!file)
      {
	cout << "keine datei" << file << "aus: " << filename << endl;
	std::string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
      }
    std::string line, path, classlabel;
    while (getline(file, line))
      {
	std::stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty())
	  {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
	  }
      }
  }

  int detectAndDisplay( cv::Mat frame, int im_width, int im_height,Ptr<cv::face::EigenFaceRecognizer> model)
  {
    // ROS_INFO_STREAM("Start detect and Display function");
    std::vector<cv::Rect> faces;
    cv::Mat frame_gray;
    Mat original = frame.clone();
    
    cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );
    
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(80, 80) );
    // ROS_INFO_STREAM("Start work with detected faces count: " << faces.size());
    cv::waitKey(700);
    
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
	// Get the prediction and associated confidence from the model
	// model->predict(face_resized, predicted_label, predicted_confidence);

	model->predict(face_resized, prediction, predicted_confidence);

	ROS_INFO_STREAM("Prediction: " << prediction << ", Confidence: " << predicted_confidence);
		if(prediction > -1)
		  {
	rectangle(original, face_i, CV_RGB(0, 255,0), 1);
	string box_text = format("Prediction = %d", prediction);
	int pos_x = std::max(face_i.tl().x - 10, 0);
	int pos_y = std::max(face_i.tl().y - 10, 0);
	putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
		  }
	else
	  {
	    ROS_INFO("not found");
	  }

	    
      }
    cv::imshow( "Capture - Face detection", original );
    return faces.size();
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
    goal_ = as_.acceptNewGoal()->detect_face;

    ROS_INFO_STREAM(action_name_ << " accepted goal: " << goal_);

    if(!face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml")) // == fn_haar
      ROS_INFO("FUCK");
    if(!eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml"))
      ROS_INFO("FUCK");
    
    fn_csv = string("/home/hanna/my_ws/src/face_learning/src/test.csv"); // pfad anpassen!!!

    ROS_INFO("Argumente in Variablen geschrieben");
    ROS_INFO_STREAM("csv: " << fn_csv);


    
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

    std::vector<cv::Mat> images;
    std::vector<int> labels;
    try
      {
        read_csv(fn_csv, images, labels);
      }
    catch (cv::Exception& e)
      {
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        // nothing more we can do
        exit(1);
      }

    int im_width = images[0].cols;
    int im_height = images[0].rows;
    
    // Ptr<cv::face::EigenFaceRecognizer> model = EigenFaceRecognizer::create();


    
    // Let's say we want to keep 10 Eigenfaces and have a threshold value of 10.0
    int num_components = 0;
    double threshold = 7000.0;
    // Then if you want to have a cv::FaceRecognizer with a confidence threshold,
    // create the concrete implementation with the appropiate parameters:
    Ptr<cv::face::EigenFaceRecognizer> model = EigenFaceRecognizer::create(num_components, threshold);

    
    trainModel(images, labels, model);
    cv::CascadeClassifier haar_cascade;

    
    haar_cascade.load("/usr/share/opencv/haarcascades/haacrcascade_frontalface_default.xml");
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
    number_of_faces_ = detectAndDisplay( cv_ptr->image, im_width, im_height, model);

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
  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;
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
