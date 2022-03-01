#include "main.h"
#include <thread>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "oakd_ros_node");
  ros::NodeHandle n("~");

  oakd_ros_class oak_handler(n);
  signal(SIGINT, signal_handler); // to exit program when ctrl+c


  ///// Generating and connecting device handler with pipeline
  dai::Device device(oak_handler.pipeline);
  
  while(!oak_handler.initialized){
    usleep(50000);
  }


  ///// setting others
  // YOLO bounding box color
  auto color = cv::Scalar(255, 0, 255);

  // IR laser, LED illuminator of PRO version
  bool if_IR_laser_emission = device.setIrLaserDotProjectorBrightness(oak_handler.IR_laser_brightness_mA);
  bool if_IR_flood_emission = device.setIrFloodLightBrightness(oak_handler.LED_illuminator_brightness_mA);

  cout << "Usb speed: " << device.getUsbSpeed() << endl;
  cout << "Laser: " << if_IR_laser_emission << " , Flood: " << if_IR_flood_emission << endl;

  ///// obtained data
  cv::Mat FrameLeft, FrameRight, FrameDepth, FrameDepthColor, FrameRgb, FrameDetect;

  ///// for point cloud
  dai::CalibrationHandler calibData = device.readCalibration();
  oak_handler.intrinsics=calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, oak_handler.depth_width, oak_handler.depth_height);
  double fx = oak_handler.intrinsics[0][0]; double cx = oak_handler.intrinsics[0][2];
  double fy = oak_handler.intrinsics[1][1]; double cy = oak_handler.intrinsics[1][2];
  


  ///// threads to get each data
  std::thread imu_thread, rgb_thread, yolo_thread, stereo_thread, depth_pcl_thread;

  if (oak_handler.get_imu){
    std::shared_ptr<dai::DataOutputQueue> imuQueue = device.getOutputQueue("imu", 50, false);
    imu_thread = std::thread([&]() {
      while(ros::ok()){
        std::shared_ptr<dai::IMUData> inPassIMU = imuQueue->tryGet<dai::IMUData>();
        if (inPassIMU != nullptr){
          for (int i = 0; i < inPassIMU->packets.size(); ++i)
          {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.frame_id = oak_handler.topic_prefix+"_frame";
            imu_msg.header.stamp = ros::Time::now();
            dai::IMUPacket imuPackets = inPassIMU->packets[i];
            
            dai::IMUReportAccelerometer accelVal = imuPackets.acceleroMeter;
            dai::IMUReportGyroscope gyro_val = imuPackets.gyroscope;
            dai::IMUReportRotationVectorWAcc rotation_val = imuPackets.rotationVector;

            imu_msg.linear_acceleration.x = accelVal.x; imu_msg.linear_acceleration.y = accelVal.y; imu_msg.linear_acceleration.z = accelVal.z;
          
            imu_msg.angular_velocity.x = gyro_val.x; imu_msg.angular_velocity.y = gyro_val.y; imu_msg.angular_velocity.z = gyro_val.z;

            imu_msg.orientation.x = rotation_val.i; imu_msg.orientation.y = rotation_val.j;
            imu_msg.orientation.z = rotation_val.k; imu_msg.orientation.w = rotation_val.real;

            oak_handler.imu_pub.publish(imu_msg);
          }
        }
        std::chrono::microseconds dura(1000);
        std::this_thread::sleep_for(dura);
      }
    });
  }


  if (oak_handler.get_rgb){
    std::shared_ptr<dai::DataOutputQueue> rgbQueue = device.getOutputQueue("rgb", 8, false);
    rgb_thread = std::thread([&]() {
      std_msgs::Header header;
      while(ros::ok()){
        std::shared_ptr<dai::ImgFrame> inPassRgb = rgbQueue->tryGet<dai::ImgFrame>();
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


  if (oak_handler.get_YOLO){
    std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = device.getOutputQueue("detections", 8, false);
    std::shared_ptr<dai::DataOutputQueue> nNetImgQueue = device.getOutputQueue("detected_img", 8, false);
    yolo_thread = std::thread([&]() {
      std_msgs::Header header;
      while(ros::ok()){
        std::shared_ptr<dai::ImgDetections> inPassNN = nNetDataQueue->tryGet<dai::ImgDetections>();
        std::shared_ptr<dai::ImgFrame> inPassNN_img = nNetImgQueue->tryGet<dai::ImgFrame>();
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
    std::shared_ptr<dai::DataOutputQueue> leftQueue = device.getOutputQueue("left", 8, false);
    std::shared_ptr<dai::DataOutputQueue> rightQueue = device.getOutputQueue("right", 8, false);
    stereo_thread = std::thread([&]() {
      std_msgs::Header header;
      while(ros::ok()){
        std::shared_ptr<dai::ImgFrame> inPassLeft = leftQueue->tryGet<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> inPassRight = rightQueue->tryGet<dai::ImgFrame>();
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
    std::shared_ptr<dai::DataOutputQueue> DepthQueue = device.getOutputQueue("depth", 8, false);
    depth_pcl_thread = std::thread([&]() {
      std_msgs::Header header;
      while(ros::ok()){
        std::shared_ptr<dai::ImgFrame> inPassDepth = DepthQueue->tryGet<dai::ImgFrame>();
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
                if (temp_depth/1000.0 >= oak_handler.pcl_min_range and temp_depth/1000.0 <= oak_handler.pcl_max_range){
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