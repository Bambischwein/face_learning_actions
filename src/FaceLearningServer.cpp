#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/aruco.hpp"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <actionlib/server/simple_action_server.h>
#include <face_learning_actions/FaceLearningAction.h>
#include <image_transport/image_transport.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace cv::face;
using namespace std;

class FaceLearningAction
{
public:

  FaceLearningAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    // register the goal feedback callbacks
    as_.registerGoalCallback(boost::bind(&FaceLearningAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FaceLearningAction::preemptCB, this));


    //subscribe to the data topic of interest
    // /kinect2/qhd/image_color , /camera/image_raw /pepper_robot/camera/front/image_raw
    image_sub_ = nh_.subscribe<sensor_msgs::Image>( "/camera/image_raw", 1, &FaceLearningAction::analysisCB, this);
    image_transport::ImageTransport it(nh_);
    detector_pub = it.advertise("detector_stream", 1000);
    as_.start();
  }

  ~FaceLearningAction(void)
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

  
  static void trainModel(std::vector<cv::Mat>& images, std::vector<int>& labels, cv::Ptr<cv::face::FisherFaceRecognizer>& model)
  {
    model->train(images, labels);
  }

  void goalCB()
  {
    // ROS_INFO_STREAM(action_name_ << " executing goalCB()");

    // ROS_INFO_STREAM(action_name_ << " accepted goal: " << goal_);


    goal_ = as_.acceptNewGoal()->save_files;

    
    face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml");

    fn_csv = string("/home/hanna/action_ws/src/face_learning_actions/src/TrainData.csv");

    image_database_path = "/home/hanna/action_ws/src/face_learning_actions/src/images/";
  }

  void preemptCB()
  {
    // ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void analysisCB(const sensor_msgs::Image::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    // ROS_INFO("analysis cb");

    std::vector<cv::Mat> images;
    std::vector<int> labels;
    std::vector<string> names;

    // read the stream 
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
    
    // read the csv file
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


      if(result_.name.empty())
      {
	ROS_INFO("Bitte Namen eintragen: ");
	cin >> result_.name;
	result_.id = labels.back() + 1;
	ROS_INFO_STREAM("id = " << result_.id);
      }
    int im_width = 200;
    int im_height = 200;
      
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    cv::waitKey(200);

    Mat face = cv_ptr->image;
    Mat face_resized;
    // hier kommt ih
	
    Mat gray;
    cvtColor(face, gray, CV_BGR2GRAY);
    // Find the faces in the frame:
    vector< Rect_<int> > faces;
    face_cascade.detectMultiScale(gray, faces);
	
    // Crop image
    Mat imCrop = face(faces[0]);
    cv::resize(imCrop, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
	    
    images.push_back(face_resized);
    labels.push_back(result_.id);

    std::stringstream var;
    var << image_database_path << "IMG00" << result_.id << "_" << feedback_.number_of_faces << ".jpg";
    std::string s = var.str();
    imwrite(s, face_resized, compression_params);
	    
    ofstream out("/home/hanna/action_ws/src/face_learning_actions/src/TrainData.csv", ios::app);
    out <<endl;
    out << s << ";" << result_.id << ";" << result_.name << endl;

    as_.publishFeedback(feedback_);
    feedback_.number_of_faces++;

    cv::waitKey(200);
    if(feedback_.number_of_faces == goal_)
      {
	// ROS_INFO("Success.");
	feedback_.number_of_faces = 0;
	result_.name = "";
	as_.setSucceeded();
      }
    else
      {
	// ROS_INFO("No success.");
	as_.setAborted();
      }
  }

protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<face_learning_actions::FaceLearningAction> as_;
  std::string action_name_;

int goal_;
  face_learning_actions::FaceLearningFeedback feedback_;
  face_learning_actions::FaceLearningResult result_;
  ros::Subscriber image_sub_;

  bool success_;
  int  img_counter_;

  /** Global variables */
  cv::VideoCapture capture;
  cv::Mat frame;
  std::string image_database_path;
  image_transport::Publisher detector_pub ;
  cv::CascadeClassifier face_cascade;
  std::string fn_csv;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FaceLearningActionServer");

  FaceLearningAction fd(ros::this_node::getName());
  ros::spin();

  return 0;
}




