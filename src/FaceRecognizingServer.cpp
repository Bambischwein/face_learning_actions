bg#include <ros/ros.h>
#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/aruco.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <actionlib/server/simple_action_server.h>
#include <face_learning_actions/FaceRecognizingAction.h>

#include <ros/console.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace cv::face;
using namespace std;

class FaceRecognizingAction
{
public:

  FaceRecognizingAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    // register the goal feedback callbacks
    as_.registerGoalCallback(boost::bind(&FaceRecognizingAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FaceRecognizingAction::preemptCB, this));

    // subscriber if neccessary
    image_sub_ = nh_.subscribe<sensor_msgs::Image>( "/camera/image_raw", 1, &FaceRecognizingAction::analysisCB, this);

    
    as_.start();
  }

  ~FaceRecognizingAction(void)
  {
  }


  static void read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, char separator = ';')
  {
    std::ifstream file(filename.c_str(), std::ifstream::in);
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


  static void trainModel(std::vector<cv::Mat>& images, std::vector<int>& labels, cv::Ptr<cv::face::FisherFaceRecognizer>& model)
  {
    model->train(images, labels);
  }

  

  void goalCB()
  {
    ROS_INFO_STREAM(action_name_ << " executing goalCB()");
    
    // reset helper variables
    number_of_faces_ = 0;

    // accept the new goal
    goal_ = as_.acceptNewGoal()->known_face;

    ROS_INFO_STREAM(action_name_ << " accepted goal: " << goal_);


    // einlesen der xml und csv!!
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void analysisCB(const sensor_msgs::Image::ConstPtr& msg)
  {
    // ROS_INFO("test");
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    ROS_INFO("analysis cb");


    // eigentliches Programm:

    // Get the path to your CSV: !!! 
    string fn_haar = string("");
    string fn_csv = string("");
    int deviceId = atoi("");
    cout << "Argumente in Variablen geschrieben" << endl;
    cout << "haarcascade: " << fn_haar  << endl;
    cout << "csv: " << fn_csv << endl;
    cout << "device id: " << deviceId << endl;
    // These vectors hold the images and corresponding labels:
    vector<Mat> images;
    vector<int> labels;
    // Read in the data (fails if no valid input filename is given, but you'll get an error message):
    try
      {
        read_csv(fn_csv, images, labels);
	cout << "CSV gelesen" << endl;
      }
    catch (cv::Exception& e)
      {
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        // nothing more we can do
        exit(1);
      }
    int im_width = images[0].cols;
    int im_height = images[0].rows;
    Ptr<FisherFaceRecognizer> model = FisherFaceRecognizer::create();
    trainModel(images, labels, model);
    CascadeClassifier haar_cascade;
    haar_cascade.load(fn_haar);
    VideoCapture cap(deviceId);
    if(!cap.isOpened())
      {
        cerr << "Capture Device ID " << deviceId << "cannot be opened." << endl;
        return;
      }
    Mat frame;
    for(;;)
      {
        cap >> frame;
        Mat original = frame.clone();
        Mat gray;
        cvtColor(original, gray, CV_BGR2GRAY);
        // Find the faces in the frame:
        vector< Rect_<int> > faces;
        haar_cascade.detectMultiScale(gray, faces);
        for(int i = 0; i < faces.size(); i++)
	  {
            // Process face by face:
            Rect face_i = faces[i];
            // Crop the face from the image. So simple with OpenCV C++: !!!
            Mat face = gray(face_i);
            Mat face_resized;
            cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);

	    int predicted_label = -1;
	    double predicted_confidence = 0.0;
	    // Get the prediction and associated confidence from the model
	    model->predict(face_resized, predicted_label, predicted_confidence);
	    ROS_INFO_STREAM("Prediction: " << predicted_label << ", Confidence: " << predicted_confidence);
            int prediction = model->predict(face_resized);
            rectangle(original, face_i, CV_RGB(0, 255,0), 1);
            string box_text = format("Prediction = %d", prediction);
            int pos_x = std::max(face_i.tl().x - 10, 0);
            int pos_y = std::max(face_i.tl().y - 10, 0);
            putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
	  }
        // Show the result:
        imshow("face_recognizer", original);
        // And display it:
        char key = (char) waitKey(20);
        // Exit this loop on escape:
        if(key == 27)
	  break;
      }


  }

protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<face_learning_actions::FaceRecognizingAction> as_;
  std::string action_name_;

  face_learning_actions::FaceRecognizingFeedback feedback_;
  face_learning_actions::FaceRecognizingResult result_;
  ros::Subscriber image_sub_;

  bool goal_;
  bool success_;
  int number_of_faces_;

    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FaceRecognizingActionServer");

  FaceRecognizingAction fd(ros::this_node::getName());
  ros::spin();

  return 0;
}




