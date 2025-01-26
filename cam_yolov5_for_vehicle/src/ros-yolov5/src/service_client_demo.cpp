//
// Created by ou on 2021/3/14.
//

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros_yolo/yolo.h>
#include <ros_yolo/yoloAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;

ros::ServiceClient client;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client_node");
  ros::NodeHandle n;

  int key = 0;
  double distance = 0.0;
  double height = 1.7;
  double k = 500;
  // 创建节点句柄
  ros::NodeHandle nh;

  // 创建图像传输对象
  image_transport::ImageTransport it(n);

  while (ros::ok())
  {
    // 获取最新的图像消息
    sensor_msgs::ImageConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw", n);
    if (msg)
    {
      try
      {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame0 = cv_ptr->image;
        cv::Mat frame; // 放大两倍
        cv::resize(frame0, frame, cv::Size(), 1.5, 1.5);
        // 在图像上绘制矩形
        // cv::rectangle(cv_ptr->image, cv::Point(50, 50), cv::Point(200, 200), CV_RGB(255, 0, 0), 2);
        // cv::putText(cv_ptr->image, std::to_string(i), cv::Point(210, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1, cv::LINE_AA);
        // if(++i==10)i=0;
        // 显示图像
        // cv::imshow("Camera View", cv_ptr->image);
        // cv::waitKey(30); // 等待30ms

        ros::ServiceClient client = n.serviceClient<ros_yolo::yolo>("yolo_service");
        client.waitForExistence(ros::Duration(30e-3));

        ros_yolo::yolo srv;
        srv.request.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        if (client.call(srv))
        {
          cout << "----------------" << endl;
          cout << "detect: " << srv.response.results.size() << " objects" << endl;
          for (auto &result : srv.response.results)
          {
            auto xyxy = result.bbox.xyxy;
            cv::Point p1(xyxy[0], xyxy[1]), p2(xyxy[2], xyxy[3]), wh = p2 - p1;
            auto thickness = cv::min(wh.x, wh.y);
            cv::rectangle(frame, p1, p2, cv::Scalar(128, 0, 128), 2);
            distance =  k * height/(result.bbox.xyxy[3] - result.bbox.xyxy[1]);
            // Format distance to one decimal place
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(1) << distance;
            std::string distance_str = stream.str();
            string result_label = result.label + " " + distance_str + " m";
            cv::putText(frame, result_label, p1, cv::FONT_HERSHEY_COMPLEX,
                        1, cv::Scalar(128, 0, 128),
                        2, 8);
            cout << result.label << endl;
            cout << result.bbox.xyxy[0] << " " << result.bbox.xyxy[1] << " " << result.bbox.xyxy[2] << " "
                 << result.bbox.xyxy[3] << endl;

            // cout << "distance: " <<fixed << setprecision(3)<< distance << "m" << endl;
            printf("distance: %.3f m\n", distance);
          }
        }
        cv::imshow("img", frame);
        key = cv::waitKey(30);
        printf("input: %d\n", key);
        if (key == 32)
        { // Press space to exit
          // 释放摄像头资源并销毁所有窗口
          printf("exit\n");
          // capture.release();
          frame.release();
          cv::destroyAllWindows();
          return 0;
        }
        ros::spinOnce();
      }
      catch (cv_bridge::Exception &e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }
  }

  // capture.release();
  cv::destroyAllWindows();
  return 0;
}