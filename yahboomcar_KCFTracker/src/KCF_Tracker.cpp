#include <iostream>
#include "KCF_Tracker.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "kcftracker.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
Rect selectRect;
Point origin;
Rect result;
bool select_flag = false;
bool bRenewROI = false;
bool bBeginKCF = false;
Mat rgbimage;
Mat depthimage;
const int &ACTION_ESC = 27;
const int &ACTION_SPACE = 32;



void onMouse(int event, int x, int y, int, void *) {
    if (select_flag) {
        selectRect.x = MIN(origin.x, x);
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);
        selectRect.height = abs(y - origin.y);
        selectRect &= Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == 1) {
//    if (event == CV_EVENT_LBUTTONDOWN) {
        bBeginKCF = false;
        select_flag = true;
        origin = Point(x, y);
        selectRect = Rect(x, y, 0, 0);
    } else if (event == 4) {
//    } else if (event == CV_EVENT_LBUTTONUP) {
        select_flag = false;
        bRenewROI = true;
    }
}

void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    linear_speed = 0;
    rotation_speed = 0;
    enable_get_depth = false;
    this->linear_PID->reset();
    this->angular_PID->reset();
    vel_pub_->publish(geometry_msgs::msg::Twist());

}

void ImageConverter::Cancel() {
    this->Reset();
    
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    //delete this->linear_PID;
    //delete this->angular_PID;
    
    destroyWindow(RGB_WINDOW);
//        destroyWindow(DEPTH_WINDOW);
}

void ImageConverter::PIDcallback() {

    this->minDist=1.0;
    this->linear_PID->Set_PID(3.0, 0.0, 1.0);
    this->angular_PID->Set_PID(0.5, 0.0, 2.0);
    this->linear_PID->reset();
    this->angular_PID->reset();
}


void ImageConverter::imageCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        std::cout<<"cv_bridge exception"<<std::endl;
        return;
    }
    
    cv_ptr->image.copyTo(rgbimage);
    setMouseCallback(RGB_WINDOW, onMouse, 0);
    if (bRenewROI) {
         if (selectRect.width <= 0 || selectRect.height <= 0)
         {
             bRenewROI = false;
             return;
         }
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }
    if (bBeginKCF) {
        result = tracker.update(rgbimage);
        rectangle(rgbimage, result, Scalar(0, 255, 255), 1, 8);
        circle(rgbimage, Point(result.x + result.width / 2, result.y + result.height / 2), 3, Scalar(0, 0, 255),-1);
    } else rectangle(rgbimage, selectRect, Scalar(255, 0, 0), 2, 8, 0);
    //sensor_msgs::ImagePtr kcf_imagemsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbimage).toImageMsg();
    //mage_pub_ -> publish(kcf_imagemsg.get());


    sensor_msgs::msg::Image kcf_imagemsg;
    std_msgs::msg::Header _header;
    cv_bridge::CvImage _cv_bridge;
    _header.stamp = this->get_clock() -> now();
    _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, rgbimage);
    _cv_bridge.toImageMsg(kcf_imagemsg);
    image_pub_-> publish(kcf_imagemsg);
    imshow(RGB_WINDOW, rgbimage);
    int action = waitKey(1) & 0xFF;
    if (action == 'q' || action == ACTION_ESC) this->Cancel();
    else if (action == 'r')  this->Reset();
    else if (action == ACTION_SPACE) enable_get_depth = true;
}

void ImageConverter::depthCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
	this->get_parameter<float>("minDist_",this->minDist);
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.copyTo(depthimage);
    }
    catch (cv_bridge::Exception &e) {
        std::cout<<"Could not convert from  to 'TYPE_32FC1'."<<std::endl;
    }
    if (enable_get_depth) {
        int center_x = (int)(result.x + result.width / 2);
        std::cout<<"center_x: "<<center_x<<std::endl;
        int center_y = (int)(result.y + result.height / 2);
        std::cout<<"center_y: "<<center_y<<std::endl;
        dist_val[0] = depthimage.at<float>(center_y - 5, center_x - 5)/1000.0;
        dist_val[1] = depthimage.at<float>(center_y - 5, center_x + 5)/1000.0;
        dist_val[2] = depthimage.at<float>(center_y + 5, center_x + 5)/1000.0;
        dist_val[3] = depthimage.at<float>(center_y + 5, center_x - 5)/1000.0;
        dist_val[4] = depthimage.at<float>(center_y, center_x);
        std::cout<<"dist_val[0]: "<<dist_val[0]<<std::endl;
        std::cout<<"dist_val[1]: "<<dist_val[1]<<std::endl;
        std::cout<<"dist_val[2]: "<<dist_val[2]<<std::endl;
        std::cout<<"dist_val[3]: "<<dist_val[3]<<std::endl;
        std::cout<<"dist_val[4]: "<<dist_val[4]<<std::endl;
        float distance = 0;
        int num_depth_points = 5;
        for (int i = 0; i < 5; i++) {
            if (dist_val[i] > 0.4 && dist_val[i] < 10.0) distance += dist_val[i];
            else num_depth_points--;
        }
        distance /= num_depth_points;
        std::cout<<distance<<std::endl;
        if (num_depth_points != 0) {
        	std::cout<<"minDist: "<<minDist<<std::endl;
            if (abs(distance - this->minDist) < 0.1) linear_speed = 0;
            else linear_speed = -linear_PID->compute(this->minDist, distance);//-linear_PID->compute(minDist, distance)
        }
        rotation_speed = angular_PID->compute(320 / 100.0, center_x / 100.0);//angular_PID->compute(320 / 100.0, center_x / 100.0)
        if (abs(rotation_speed) < 0.1)rotation_speed = 0;
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_speed;
        twist.angular.z = rotation_speed;
        vel_pub_->publish(twist);
        
    }
    else{
    	geometry_msgs::msg::Twist twist;
    	vel_pub_->publish(twist);
    }
//        imshow(DEPTH_WINDOW, depthimage);
    waitKey(1);
}

void ImageConverter::JoyCb(const std::shared_ptr<std_msgs::msg::Bool> msg) {
    enable_get_depth = msg->data;
}








