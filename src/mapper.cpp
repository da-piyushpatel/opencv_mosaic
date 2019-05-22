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

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

cv::Mat imgframe;

Mat cal_homography_matrix(Mat image1, Mat image2, int features, int layers) {

	Ptr<ORB> detector = ORB::create(features);

	std::vector<KeyPoint> keypoints_1, keypoints_2;

	detector->detect(image1, keypoints_1);
	detector->detect(image2, keypoints_2);

	// //-- Draw keypoints
	// Mat img_keypoints_1; Mat img_keypoints_2;

	// drawKeypoints(image1, keypoints_1, img_keypoints_1, Scalar::all(-1),
	// 	DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	// drawKeypoints(image2, keypoints_2, img_keypoints_2, Scalar::all(-1),
	// 	DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // //-- Show detected (drawn) keypoints

	// imshow("Keypoints 1", img_keypoints_1);
	// imshow("Keypoints 2", img_keypoints_2);

	//-- Step 2: Calculate descriptors (feature vectors)
	Mat descriptors_1, descriptors_2;
	detector->compute(image1, keypoints_1, descriptors_1);
	detector->compute(image2, keypoints_2, descriptors_2);


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


	// Find the Homography Matrix for img 1 and img2
	H = findHomography(obj, scene, CV_RANSAC);
	//H is the transformation. Using ransac to remove the outliers
	image1.release();
	image2.release();
	return H;
}
Mat stitch_image(Mat image1, Mat image2, Mat H)
{


	cv::Mat result;
	warpPerspective(image1, result, H, cv::Size(image1.cols + image2.cols, image1.rows+image2.rows));
	// cv::imshow("image1",image1);
    // cv::imshow("result",result);
	cv::Mat half(result, cv::Rect(0, 0, image2.cols, image2.rows));
    // cv::imshow("half",half);
	image2.copyTo(half);
	return result;

}

void ImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		// framegrab.frameID = ::count++;
		// cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
		// cv::waitKey(30);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception:%s",e.what());
        return;
    }
    imgframe = cv_ptr->image;
        // cv::Mat img_rgb = cv_ptr->image;
			// framegrab.rgb = img_rgb.clone();
            // cv::imshow("img",framegrab.rgb);
};

using namespace cv;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"realtimemosaic");
    ros::NodeHandle nh_; 
    image_transport::ImageTransport it_(nh_);
    // image_transport::Subscriber image_sub_;
	int64 t0 = cv::getTickCount();

	int nOctaveLayers = 3; //lowes paper suggests
	int nfeatures = 500; //number of features


    // VideoCapture cap(0);

    // if(!cap.isOpened()){
    //     cout<<"Error while reading input file"<<endl;
    //     return -1;  
    // }

    image_transport::Subscriber image_sub_ = it_.subscribe("/mv/image_raw",1,ImgCallback);

    // cap.set(CAP_PROP_POS_MSEC,1000);

    Mat currframe, result, homo;
    int count =0;
    ros::Rate loop_rate(1);

    while(ros::ok()){
        // printf("1");
        if(count == 0)
        {
            // cap >> result;
            // cv::cvtColor(result,result,COLOR_BGR2GRAY);
          ImgCallback;
          result = imgframe;
	        cv::cvtColor(result,result,COLOR_BGR2GRAY);

        }
        ImgCallback;

        currframe = imgframe;
        // printf("2");
        if(currframe.empty())
    		break;

	    // currframe.convertTo(currframe,CV_8UC1);
        cv::cvtColor(currframe,currframe,COLOR_BGR2GRAY);
        
		// printf("3");
        homo = cal_homography_matrix(result, currframe, 500, 3);
        result = stitch_image(result, currframe, homo);

        // cout<<cv::getTickCount()-t0<<endl;

		    currframe.release();
        count++;
        // cout<<"count"<<count<<endl;
        imshow("Stitched Image", result);
        char c=(char)waitKey(1);
        if(c==27)
            break;
        ros::spinOnce();
        loop_rate.sleep();

    }

	imshow("Stitched Image", result);
	// cvtColor(result_t, result_t, COLOR_RGB2GRAY);
	int64 t1 = cv::getTickCount();
	double secs = (t1 - t0) / cv::getTickFrequency();
	cout << "Time elapsed:" << secs << "seconds";
	imwrite("result.jpg", result);
	
    
    waitKey(0);
	return 0;
}