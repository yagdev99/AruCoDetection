#include <ros/ros.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/CameraInfo.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/TransformStamped.h>

// Camera Parameters
// float K[3][3] = {{322.0704122808738, 0, 199.2680620421962},{0, 320.8673986158544, 155.2533082600705},{0,0,1}};
// float D[5] = {0,0,0,0,0};



void img_cb(const sensor_msgs::ImageConstPtr & msg){

    // Camera Parameters 
    cv::Mat cameraMatrix; // = cv::Mat(3, 3, CV_64FC1, K);
    cv::Mat distCoeffs;// = cv::Mat(5,1,CV_64FC1,D);

    // transform broadcaster
    static tf2_ros::TransformBroadcaster br;    

    /**  Converts Image message to cv image */
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){ 
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Converting Image message to CV Image
    cv::Mat image;
    cv::Mat color_img = cv_ptr->image;
    cv::cvtColor(color_img,image,cv::COLOR_BGR2GRAY);

    // Defining the Matrix and distCoeff
    cv::Size imgSize = image.size();
    cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
    cameraMatrix.at<double>(0, 0) = cameraMatrix.at<double>(1, 1) = 650;
    cameraMatrix.at<double>(0, 2) = imgSize.width / 2;
    cameraMatrix.at<double>(1, 2) = imgSize.height / 2;
    distCoeffs = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

    // Selecting Type of aruco tags
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);


    // Detecting the markers
    cv::Mat imageCopy;
    cv::Mat imgClrCopy;

    color_img.copyTo(imgClrCopy);
    
    // TODO: Add CLAHE 

    image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids);

    std::cout << ids.size() << std::endl;


    // Drawing the axis 
    if (ids.size() > 0){
        
        cv::aruco::drawDetectedMarkers(imgClrCopy, corners, ids);

        // std::cout << corners[0] << std::endl;
        
        // Rotation and Translation Vector for the aruco markers
        std::vector<cv::Vec3d> rvecs, tvecs;

        // Estimating the pose of the markers
        cv::aruco::estimatePoseSingleMarkers(corners, 0.2, cameraMatrix, distCoeffs, rvecs, tvecs);
         
        // draw axis for each marker
        // std::cout << "Starting: " << std::endl;
        for(int i=0; i<ids.size(); i++){

            // std::cout << rvecs[i] << " " << tvecs[i];
            cv::Vec3d rvec = rvecs[i];
            cv::Vec3d tvec = tvecs[i];

            // Transform to be published
            geometry_msgs::TransformStamped ts;

            tf2::Quaternion q;
            q.setRPY(rvec[0],rvec[1],rvec[2]);

            ts.header.stamp = ros::Time::now();
            ts.child_frame_id = "Frame_" + std::to_string(ids[i]);
            ts.header.frame_id = "base_footprint";

            ts.transform.translation.x = tvec[0];
            ts.transform.translation.y = tvec[1];
            ts.transform.translation.z = tvec[2];

            ts.transform.rotation.x = q.x();
            ts.transform.rotation.y = q.y();
            ts.transform.rotation.z = q.z();
            ts.transform.rotation.w = q.w();

            br.sendTransform(ts);

            cv::aruco::drawAxis(imgClrCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

        }
        // std::cout << "--------------" << std::endl;
    }
    
    // Displaying the output
    cv::imshow("out", imgClrCopy);
    cv::waitKey(10);
  
}


int main(int argc, char ** argv){

    ros::init(argc,argv,"aruco_detector_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw",100,img_cb);
    ros::spin();


}