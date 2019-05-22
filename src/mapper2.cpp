#define _CRT_SECURE_NO_DEPRECATE
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/stitching.hpp"
#include <ros/ros.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

cv::Mat imgframe;
cv::Mat currframe, result;
int count =0;

class ImageConverter{

ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;

public:
	ImageConverter()
	:it_(nh_)
	{
		image_sub_ = it_.subscribe("/mv/image_raw",1,& ImageConverter::imageCb,this);
	}

	 void imageCb(const sensor_msgs::ImageConstPtr& msg)
  	{
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

		if(::count++ == 0)
		{
			result = cv_ptr->image;
			cv::cvtColor(result,result,COLOR_BGR2GRAY);
		}
		::count++;
		currframe = cv_ptr->image;
		cv::cvtColor(currframe,currframe,COLOR_BGR2GRAY);
	
	//mosiacing starts
	Ptr<ORB> detector = ORB::create(500);

	std::vector<KeyPoint> keypoints_1, keypoints_2;

	detector->detect(result, keypoints_1);
	detector->detect(currframe, keypoints_2);

	Mat descriptors_1, descriptors_2;
	detector->compute(result, keypoints_1, descriptors_1);
	detector->compute(currframe, keypoints_2, descriptors_2);


	//-- Step 3: Matching descriptor vectors using BFMatcher :
	BFMatcher matcher;
	std::vector<std::vector<cv::DMatch> > matches;
	
	matcher.knnMatch(descriptors_1, descriptors_2, matches, 2);  // Find two nearest matches

	vector<cv::DMatch> good_matches;
	for (int i = 0; i < matches.size(); ++i)
	{
		const float ratio = 0.8; // As in Lowe's paper; can be tuned
		if (matches[i][0].distance < ratio * matches[i][1].distance)
		{
			good_matches.push_back(matches[i][0]);
		}
	}

	cv::Mat H;
	std::vector< Point2f > obj;
	std::vector< Point2f > scene;

	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		obj.push_back(keypoints_1[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_2[good_matches[i].trainIdx].pt);
	}


	H = findHomography(obj, scene, CV_RANSAC);

	warpPerspective(result, result, H, cv::Size(result.cols + currframe.cols, result.rows+currframe.rows));
	cv::Mat half(result, cv::Rect(0, 0, currframe.cols, currframe.rows));
    // cv::imshow("half",half);
	currframe.copyTo(half);
	cv::imshow("img",result);
	cv::waitKey(1);

}
};

// Mat cal_homography_matrix(Mat image1, Mat image2, int features, int layers) {

// 	Ptr<ORB> detector = ORB::create(features);

// 	std::vector<KeyPoint> keypoints_1, keypoints_2;

// 	detector->detect(image1, keypoints_1);
// 	detector->detect(image2, keypoints_2);

// 	Mat descriptors_1, descriptors_2;
// 	detector->compute(image1, keypoints_1, descriptors_1);
// 	detector->compute(image2, keypoints_2, descriptors_2);


// 	//-- Step 3: Matching descriptor vectors using BFMatcher :
// 	BFMatcher matcher;
// 	std::vector<std::vector<cv::DMatch> > matches;
	
// 	matcher.knnMatch(descriptors_1, descriptors_2, matches, 2);  // Find two nearest matches

// 	vector<cv::DMatch> good_matches;
// 	for (int i = 0; i < matches.size(); ++i)
// 	{
// 		const float ratio = 0.8; // As in Lowe's paper; can be tuned
// 		if (matches[i][0].distance < ratio * matches[i][1].distance)
// 		{
// 			good_matches.push_back(matches[i][0]);
// 		}
// 	}

// 	cv::Mat H;
// 	std::vector< Point2f > obj;
// 	std::vector< Point2f > scene;

// 	for (int i = 0; i < good_matches.size(); i++)
// 	{
// 		//-- Get the keypoints from the good matches
// 		obj.push_back(keypoints_1[good_matches[i].queryIdx].pt);
// 		scene.push_back(keypoints_2[good_matches[i].trainIdx].pt);
// 	}


// 	H = findHomography(obj, scene, CV_RANSAC);
// 	image1.release();
// 	image2.release();
// 	return H;
// }
// Mat stitch_image(Mat image1, Mat image2, Mat H)
// {


// 	cv::Mat result;
// 	warpPerspective(image1, result, H, cv::Size(image1.cols + image2.cols, image1.rows+image2.rows));
// 	cv::Mat half(result, cv::Rect(0, 0, image2.cols, image2.rows));
//     image2.copyTo(half);
// 	return result;

// }

// void ImgCallback(const sensor_msgs::ImageConstPtr& msg)
// {
// 	cv_bridge::CvImagePtr cv_ptr;
//     try{
//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch(cv_bridge::Exception& e){
//         ROS_ERROR("cv_bridge exception:%s",e.what());
//         return;
//     }
//     imgframe = cv_ptr->image;

// };

int main(int argc, char** argv)
{
    ros::init(argc,argv,"realtimemosaic");
    // ros::NodeHandle nh_; 
    // image_transport::ImageTransport it_(nh_);
    // image_transport::Subscriber image_sub_;
	// int64 t0 = cv::getTickCount();

	// int nOctaveLayers = 3; //lowes paper suggests
	// int nfeatures = 500; //number of features

    // image_transport::Subscriber image_sub_ = it_.subscribe("/mv/image_raw",1,ImgCallback);
 
    // cap.set(CAP_PROP_POS_MSEC,1000);

    // Mat currframe, result, homo;
    // int count =0;
    // ros::Rate loop_rate(1);

    // while(ros::ok()){
    //     if(count == 0)
    //     {
    //       ImgCallback;
    //       result = imgframe;
    //       cv::cvtColor(result,result,COLOR_BGR2GRAY);

    //     }
    //     ImgCallback;

    //     currframe = imgframe;
    //     if(currframe.empty())
    // 		break;

	//     cv::cvtColor(currframe,currframe,COLOR_BGR2GRAY);
        
	//     homo = cal_homography_matrix(result, currframe, 500, 3);
    //     result = stitch_image(result, currframe, homo);

    //     // cout<<cv::getTickCount()-t0<<endl;

	// 	    currframe.release();
    //     count++;
    //     // cout<<"count"<<count<<endl;
    //     imshow("Stitched Image", result);
    //     char c=(char)waitKey(1);
    //     if(c==27)
    //         break;
    //     ros::spinOnce();
    //     loop_rate.sleep();

    // }

	// imshow("Stitched Image", result);
	// // cvtColor(result_t, result_t, COLOR_RGB2GRAY);
	// int64 t1 = cv::getTickCount();
	// double secs = (t1 - t0) / cv::getTickFrequency();
	// cout << "Time elapsed:" << secs << "seconds";
	// imwrite("result.jpg", result);

	ImageConverter ic;
	ros::Rate loop_rate(1);
	while(ros::ok())
	{
  		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}