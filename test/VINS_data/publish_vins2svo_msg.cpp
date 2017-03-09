
// This file is part of REMODE - REgularized MOnocular Depth Estimation.
//
// Copyright (C) 2014 Matia Pizzoli <matia dot pizzoli at gmail dot com>
// Robotics and Perception Group, University of Zurich, Switzerland
// http://rpg.ifi.uzh.ch
//
// REMODE is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// REMODE is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <svo_msgs/DenseInput.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "utils.h"

ros::Publisher publisher;
double min_dep,max_dep;
Eigen::Quaterniond cropped_img_quat;  // w,x,y,z
std::string result_path;

void vins_callback(const sensor_msgs::ImageConstPtr& img_msg, const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
    svo_msgs::DenseInput msg;
    msg.header.stamp = img_msg->header.stamp;
    msg.header.frame_id = "/dense_input_frame_id";

    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_8UC1);
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = img_msg->header.stamp;
    cv_image.header.frame_id = "/greyscale_image_frame_id";
    cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    cv_image.image = img_ptr->image;
    msg.image = *cv_image.toImageMsg();

    Eigen::Quaterniond q1(pose_msg->pose.orientation.w,
                          pose_msg->pose.orientation.x,
                          pose_msg->pose.orientation.y,
                          pose_msg->pose.orientation.z
                         );  // (w,x,y,z)
    Eigen::Quaterniond q2 = q1 * cropped_img_quat.inverse();
    q2.normalize();

    msg.pose = pose_msg->pose;
    msg.pose.orientation.w = q2.w();
    msg.pose.orientation.x = q2.x();
    msg.pose.orientation.y = q2.y();
    msg.pose.orientation.z = q2.z();

    msg.min_depth = min_dep;
    msg.max_depth = max_dep;

    publisher.publish(msg);

    ROS_INFO("Receive & Publish: stamp = %lf",msg.header.stamp.toSec());
}

void depth_callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

    // save depth & depth_visual
    cv::Mat depth = depth_ptr->image;
    int w,h;
    w = depth.cols;
    h = depth.rows;
    cv::Mat depth_gray = cv::Mat::zeros(h,w,CV_8UC1);

    char name[100];
    sprintf(name,"%s/%.0lf.depth", result_path.c_str(), depth_ptr->header.stamp.toSec()*100.0);
    FILE *file = fopen(name,"w");

    for (int u=0;u<w;u++)
        for (int v=0;v<h;v++)
        {
            float dep = depth.at<float>(v,u);
            fprintf(file,"%f ",dep);
            if (dep <= max_dep)
            {
                depth_gray.at<uchar>(v,u) = (uchar) (dep*255.0/max_dep);
            }
        }
    fclose(file);
    cv::Mat depth_color;
    cv::applyColorMap(depth_gray, depth_color, cv::COLORMAP_JET);

    // save image
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_8UC1);
    char img_name[100];
    sprintf(img_name,"%s/%.0lf_depth.png", result_path.c_str(), img_ptr->header.stamp.toSec()*100.0);
    cv::imwrite(img_name, depth_color);
    sprintf(img_name,"%s/%.0lf.png", result_path.c_str(), img_ptr->header.stamp.toSec()*100.0);
    cv::imwrite(img_name, img_ptr->image);
    // --------------

    ROS_WARN("Get rmd_depth: %lf",depth_ptr->header.stamp.toSec());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_publisher");
    ros::NodeHandle nh("~");

    std::string image_topic, pose_topic;

    min_dep = readParam<double>(nh, "min_depth");
    max_dep = readParam<double>(nh, "max_depth");
    image_topic = readParam<std::string>(nh, "image_topic");
    pose_topic = readParam<std::string>(nh, "pose_topic");
    cropped_img_quat.w() = readParam<double>(nh, "cropped_quat_w");
    cropped_img_quat.x() = readParam<double>(nh, "cropped_quat_x");
    cropped_img_quat.y() = readParam<double>(nh, "cropped_quat_y");
    cropped_img_quat.z() = readParam<double>(nh, "cropped_quat_z");
    result_path = readParam<std::string>(nh, "result_path");

    publisher = nh.advertise<svo_msgs::DenseInput>("/svo/dense_input",1);

    // ---- sync subscribe ----
    message_filters::Subscriber<sensor_msgs::Image> sub_img(nh,image_topic,1000);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose(nh,pose_topic,1000);
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, geometry_msgs::PoseStamped> ExactPolicy;
    message_filters::Synchronizer<ExactPolicy> sync(ExactPolicy(1000), sub_img, sub_pose);
    sync.registerCallback(boost::bind(&vins_callback, _1, _2));
    // ------------------------

    // subscribe image & remode depth
    message_filters::Subscriber<sensor_msgs::Image> sub_img_2(nh,image_topic,1000);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(nh,"/remode/depth",1000);
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy_2;
    message_filters::Synchronizer<ExactPolicy_2> sync_2(ExactPolicy_2(1000), sub_img_2, sub_depth);
    sync_2.registerCallback(boost::bind(&depth_callback, _1, _2));
    // ------------------------

    ros::spin();

    return EXIT_SUCCESS;
}
