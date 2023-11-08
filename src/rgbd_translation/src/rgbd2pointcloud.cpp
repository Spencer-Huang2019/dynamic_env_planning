//
// Created by spencer on 11/6/23.
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr cloud(new PointCloud);

void onMouse(int event, int x, int y, int flags, void* param)  //evnet:鼠标事件类型 x,y:鼠标坐标 flags：鼠标哪个键
{
  Mat* im = reinterpret_cast<Mat*>(param);
  switch (event) {

    case EVENT_LBUTTONDOWN: //EVENT_LBUTTONDOWN的意思是按下鼠标左键会触发下列程序，若改成EVENT_RBUTTONDOWN，那就是按下右键时触发程序。
      //显示图像像素值
      cout << "channel num: " << static_cast<int>(im->channels()) << endl;
      if (static_cast<int>(im->channels()) == 1)
      {
        //若图像为单通道图像，则显示鼠标点击的坐标以及灰度值
        switch (im->type())
        {
          case 0:
            cout << "at (" << x << ", " << y << " ) value is: " << static_cast<int>(im->at<uchar>(Point(x, y))) << endl; break;
          case 1:
            cout << "at (" << x << ", " << y << " ) value is: " << static_cast<int>(im->at<char>(Point(x, y))) << endl; break;
          case 2:
            cout << "at (" << x << ", " << y << " ) value is: " << static_cast<int>(im->at<ushort>(Point(x, y))) << endl; break;
          case 3:
            cout << "at (" << x << ", " << y << " ) value is: " << static_cast<int>(im->at<short>(Point(x, y))) << endl; break;
          case 4:
            cout << "at (" << x << ", " << y << " ) value is: " << static_cast<int>(im->at<int>(Point(x, y))) << endl; break;
          case 5:
            cout << "at (" << x << ", " << y << " ) value is: " << static_cast<int>(im->at<float>(Point(x, y))) << endl; break;
          case 6:
            cout << "at (" << x << ", " << y << " ) value is: " << static_cast<int>(im->at<double>(Point(x, y))) << endl; break;
        }
      }
      else
      {
        //若图像为彩色图像，则显示鼠标点击坐标以及对应的B, G, R值
        cout << "at (" << x << ", " << y << ")"
             << "  B value is: " << static_cast<int>(im->at<Vec3b>(Point(x, y))[0])
             << "  G value is: " << static_cast<int>(im->at<Vec3b>(Point(x, y))[1])
             << "  R value is: " << static_cast<int>(im->at<Vec3b>(Point(x, y))[2])
             << endl;
      }

      break;
  }
}

void colorCallback(const sensor_msgs::Image::ConstPtr& color_msg)
{
//  cout << "process rgbCallback" << endl;
  try
  {
    color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
    cv::waitKey(1050); // ms  keep updating image
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
  }
  color_pic = color_ptr->image;
}

void depthCallback(const sensor_msgs::Image::ConstPtr& depth_msg)
{
  cv::Mat img2;
//  ROS_INFO("Process depth image.");
  try
  {
    depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::waitKey(1050); // ms  keep updating image
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '32fc1'.", depth_msg->encoding.c_str());
  }
  depth_pic = depth_ptr->image;
  cout << depth_pic.type() << endl;
//  cv::imshow("show", depth_pic);
//  cv::setMouseCallback("show", onMouse, reinterpret_cast<void*>(&depth_pic));
//  cv::waitKey(0);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rgbd2pointcloud");
  ros::NodeHandle nh;


  // camera parameter
  const double camera_factor = 1;
  const double minScanDistance=0.01;
  const double maxScanDistance=2;

  // 64 * 64
  const int r = 64;
  const double camera_cx = 32.0;
  const double camera_cy = 32.0;
  const double camera_fx = 55.4256;
  const double camera_fy = 55.4256;

  // 1024 * 1024
//  const int r = 1024;
//  const double camera_cx = 512.0;
//  const double camera_cy = 512.0;
//  const double camera_fx = 886.810;
//  const double camera_fy = 886.810;

  // image subscriber
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber imgRgbSub = it.subscribe("/camera/color/image_raw", 1, colorCallback);
  image_transport::Subscriber imgDepthSub = it.subscribe("/camera/depth/image_rect_raw", 1, depthCallback);
  ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/output", 1);

  sensor_msgs::PointCloud2 pub_pointcloud;

  double sample_rate = 1.0;
  ros::Rate naptime(sample_rate);

  while (ros::ok())
  {
//    ROS_INFO("Start collecting cloud.");

//    ROS_INFO("depth_pic rows: %d", depth_pic.rows);
//    ROS_INFO("depth_pic cols: %d", depth_pic.cols);
    for (int v = 0; v < depth_pic.rows; ++v) { // axle-y
      for (int u = 0; u < depth_pic.cols; ++u) { // axle-x
        float d = depth_pic.ptr<float>(v)[u];

        if (d <= 1e-6 || d == 0)
          continue;

        pcl::PointXYZRGB p;

        // ###### camera coordinate is same with world coordinate in respect of pose
        // compute pixel (u, v) to camera (xc, yc, zc)
        double ux = u - camera_cx, vy = v - camera_cy;
        double distance = (double(d) * (maxScanDistance - minScanDistance) + minScanDistance) / camera_factor;  // unit of d value is mm
//        double zc = distance * camera_fx * camera_fy / sqrt(camera_fx * camera_fy
//           + pow(ux, 2) * pow(camera_fx, 2) + pow(vy, 2) * pow(camera_fy, 2));
        double zc = distance;
        double xc = ux * zc / camera_fx;
        double yc = vy * zc / camera_fy;
        p.z = zc + 0.15 / camera_factor;
        p.x = xc + 0.425 / camera_factor;
        p.y = yc;

        if (distance < 1.9)
          ROS_INFO("num: %d,   distance = %lf, (px, py, pz) = (%lf, %lf, %lf)", (v - 1) * r + u, distance, p.x, p.y, p.z);
        
        p.b = color_pic.ptr<uchar>(v)[u * 3];
        p.g = color_pic.ptr<uchar>(v)[u * 3 + 1];
        p.r = color_pic.ptr<uchar>(v)[u * 3 + 2];

        cloud->points.push_back(p);
      }
    }
//    ROS_INFO("Finish collecting cloud.");

    cloud->height = 1;
    cloud->width = cloud->points.size();
    ROS_INFO("point cloud size = %d", cloud->width);
    cloud->is_dense = false;
    pcl::toROSMsg(*cloud, pub_pointcloud);
    pub_pointcloud.header.frame_id = "camera_link";
    pub_pointcloud.header.stamp = ros::Time::now();

    pointcloud_publisher.publish(pub_pointcloud);
    cloud->points.clear();

    ros::spinOnce();
//    ros::spin();
    naptime.sleep();
  }
}