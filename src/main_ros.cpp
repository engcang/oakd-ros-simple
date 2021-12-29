#include "main.h"
#include <thread>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oakd_ros_node");
  ros::NodeHandle n("~");

  oakd_ros_class oak_handler(n);
  signal(SIGINT, signal_handler); // to exit program when ctrl+c



  dai::Device device(oak_handler.pipeline);
  cout << "Usb speed: " << device.getUsbSpeed() << endl;

  cv::Mat FrameLeft, FrameRight, FrameDepth, FrameDepthColor, FrameRgb, FrameDetect;
  auto color = cv::Scalar(0, 255, 0);

  /// for point cloud
  dai::CalibrationHandler calibData = device.readCalibration();
  oak_handler.intrinsics=calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, oak_handler.monoRight->getResolutionWidth(), oak_handler.monoRight->getResolutionHeight());

  double fx = oak_handler.intrinsics[0][0]; double cx = oak_handler.intrinsics[0][2];
  double fy = oak_handler.intrinsics[1][1]; double cy = oak_handler.intrinsics[1][2];
  // for(auto row : oak_handler.intrinsics) {
  //     for(auto val : row) cout << val << "  ";
  //     cout << endl;
  // }

  while(!oak_handler.initialized){
    usleep(50000);
  }
  ////////// auto can be used
  oak_handler.imuQueue = device.getOutputQueue("imu", 30, false);
  oak_handler.leftQueue = device.getOutputQueue("left", 30, false);
  oak_handler.rightQueue = device.getOutputQueue("right", 30, false);
  oak_handler.DepthQueue = device.getOutputQueue("depth", 30, false);
  oak_handler.rgbQueue = device.getOutputQueue("rgb", 30, false);
  oak_handler.nNetDataQueue = device.getOutputQueue("detections", 30, false);
  oak_handler.nNetImgQueue = device.getOutputQueue("detected_img", 30, false);

  std::thread imu_thread, rgb_thread, yolo_thread, stereo_thread, depth_pcl_thread;
  if (oak_handler.get_imu){
    imu_thread = std::thread([&]() {
      while(ros::ok()){
        std::shared_ptr<dai::IMUData> inPassIMU = oak_handler.imuQueue->tryGet<dai::IMUData>();
        if (inPassIMU != nullptr){
          sensor_msgs::Imu imu_msg;
          imu_msg.header.stamp = ros::Time::now();
          
          dai::IMUPacket imuPackets = inPassIMU->packets[inPassIMU->packets.size() - 1];
          
          dai::IMUReportAccelerometer accelVal = imuPackets.acceleroMeter;
          dai::IMUReportGyroscope gyro_val = imuPackets.gyroscope;
          dai::IMUReportRotationVectorWAcc rotation_val = imuPackets.rotationVector;

          imu_msg.linear_acceleration.x = accelVal.x; imu_msg.linear_acceleration.y = accelVal.y; imu_msg.linear_acceleration.z = accelVal.z;
        
          imu_msg.angular_velocity.x = gyro_val.x; imu_msg.angular_velocity.y = gyro_val.y; imu_msg.angular_velocity.z = gyro_val.z;

          imu_msg.orientation.x = rotation_val.i; imu_msg.orientation.y = rotation_val.j;
          imu_msg.orientation.z = rotation_val.k; imu_msg.orientation.w = rotation_val.real;

          oak_handler.imu_pub.publish(imu_msg);
        }

        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
      }
    });
  }


  if (oak_handler.get_rgb){
    rgb_thread = std::thread([&]() {
      std_msgs::Header header;
      while(ros::ok()){
        std::shared_ptr<dai::ImgFrame> inPassRgb = oak_handler.rgbQueue->tryGet<dai::ImgFrame>();
        if (inPassRgb != nullptr){
          FrameRgb = inPassRgb->getCvFrame(); // important
          header.stamp = ros::Time::now();
          cv_bridge::CvImage bridge_rgb = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, FrameRgb);
          if (oak_handler.get_raw){
            bridge_rgb.toImageMsg(oak_handler.rgb_img_msg);
            oak_handler.rgb_pub.publish(oak_handler.rgb_img_msg);
          }
          if (oak_handler.get_compressed){
            bridge_rgb.toCompressedImageMsg(oak_handler.rgb_img_comp_msg);
            oak_handler.rgb_comp_pub.publish(oak_handler.rgb_img_comp_msg);
          }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
      }     
    });
  }

  if (oak_handler.get_rgb && oak_handler.get_YOLO){
    yolo_thread = std::thread([&]() {
      std_msgs::Header header;
      while(ros::ok()){
        std::shared_ptr<dai::ImgDetections> inPassNN = oak_handler.nNetDataQueue->tryGet<dai::ImgDetections>();
        std::shared_ptr<dai::ImgFrame> inPassNN_img = oak_handler.nNetImgQueue->tryGet<dai::ImgFrame>();
        oakd_ros::bboxes bboxes_msg;
        if (inPassNN_img != nullptr ){
          FrameDetect = inPassNN_img->getCvFrame();
          header.stamp = ros::Time::now();
          if (inPassNN != nullptr){
            std::vector<dai::ImgDetection> detections = inPassNN->detections;
            for(auto& detection : detections) {
              int x1 = detection.xmin * FrameDetect.cols;
              int y1 = detection.ymin * FrameDetect.rows;
              int x2 = detection.xmax * FrameDetect.cols;
              int y2 = detection.ymax * FrameDetect.rows;

              std::string labelStr = to_string(detection.label);
              if(detection.label < oak_handler.class_names.size()) {
                  labelStr = oak_handler.class_names[detection.label];
              }
              cv::putText(FrameDetect, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
              std::stringstream confStr;
              confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
              cv::putText(FrameDetect, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
              cv::rectangle(FrameDetect, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
              
              oakd_ros::bbox box;
              box.score=detection.confidence; box.id = detection.label; box.Class = oak_handler.class_names[detection.label];
              box.x = x1; box.y = y1; box.width = x2-x1; box.height = y2-y1;
              bboxes_msg.bboxes.push_back(box);                
            }
            oak_handler.nn_bbox_pub.publish(bboxes_msg);
          }
          cv_bridge::CvImage bridge_nn = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, FrameDetect);
          if (oak_handler.get_raw){
            bridge_nn.toImageMsg(oak_handler.nn_img_msg);
            oak_handler.nn_pub.publish(oak_handler.nn_img_msg);
          }
          if (oak_handler.get_compressed){
            bridge_nn.toCompressedImageMsg(oak_handler.nn_img_comp_msg);
            oak_handler.nn_comp_pub.publish(oak_handler.nn_img_comp_msg);
          }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
      }
    });
  }


  if (oak_handler.get_stereo_ir){
    stereo_thread = std::thread([&]() {
      std_msgs::Header header;
      while(ros::ok()){
        std::shared_ptr<dai::ImgFrame> inPassLeft = oak_handler.leftQueue->tryGet<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> inPassRight = oak_handler.rightQueue->tryGet<dai::ImgFrame>();
        header.stamp = ros::Time::now();
        if (inPassLeft != nullptr){
          FrameLeft = inPassLeft->getFrame();
          cv_bridge::CvImage bridge_left = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, FrameLeft);
          if (oak_handler.get_raw){
            bridge_left.toImageMsg(oak_handler.l_img_msg);
            oak_handler.l_pub.publish(oak_handler.l_img_msg);
          }
          if (oak_handler.get_compressed){
            bridge_left.toCompressedImageMsg(oak_handler.l_img_comp_msg);
            oak_handler.l_comp_pub.publish(oak_handler.l_img_comp_msg);
          }
        }
        if (inPassRight != nullptr){
          FrameRight = inPassRight->getFrame();
          cv_bridge::CvImage bridge_right = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, FrameRight);
          if (oak_handler.get_raw){
            bridge_right.toImageMsg(oak_handler.r_img_msg);
            oak_handler.r_pub.publish(oak_handler.r_img_msg);
          }
          if (oak_handler.get_compressed){
            bridge_right.toCompressedImageMsg(oak_handler.r_img_comp_msg);
            oak_handler.r_comp_pub.publish(oak_handler.r_img_comp_msg);
          }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
      }
    });
  }

  if (oak_handler.get_stereo_depth || oak_handler.get_pointcloud){
    depth_pcl_thread = std::thread([&]() {
      std_msgs::Header header;
      while(ros::ok()){
        std::shared_ptr<dai::ImgFrame> inPassDepth = oak_handler.DepthQueue->tryGet<dai::ImgFrame>();
        if (inPassDepth != nullptr){
          FrameDepth = inPassDepth->getFrame();
          header.stamp = ros::Time::now();
          if (oak_handler.get_stereo_depth){
            cv_bridge::CvImage bridge_depth = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, FrameDepth);
            bridge_depth.toImageMsg(oak_handler.depth_img_msg);
            oak_handler.d_pub.publish(oak_handler.depth_img_msg);
          }
          if (oak_handler.get_pointcloud){
            pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl;
            for (int i = 0; i < FrameDepth.rows; ++i){
              for (int j = 0; j < FrameDepth.cols; ++j){
                float temp_depth = FrameDepth.at<ushort>(i,j);
                if (temp_depth/1000.0 >= 0.08 and temp_depth/1000.0 <= oak_handler.pcl_max_range){
                  pcl::PointXYZ p3d;
                  p3d.z = (temp_depth/1000.0); //float!!! double makes error here!!! because encoding is "32FC", float
                  p3d.x = ( j - cx ) * p3d.z / fx;
                  p3d.y = ( i - cy ) * p3d.z / fy;
                  depth_cvt_pcl.push_back(p3d);
                }
              }
            }
            oak_handler.pcl_pub.publish(cloud2msg(depth_cvt_pcl, oak_handler.topic_prefix+"_frame"));
          }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
      }
    });
  }


  imu_thread.join();
  rgb_thread.join();
  yolo_thread.join();
  stereo_thread.join();
  depth_pcl_thread.join();

  ros::spin();

  return 0;
}