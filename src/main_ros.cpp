#include "main.h"


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
  // oak_handler.intrinsics=calibData.getCameraIntrinsics(dai::CameraBoardSocket::RGB, oak_handler.camRgb->getIspWidth(), oak_handler.camRgb->getIspHeight());
  oak_handler.intrinsics=calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, oak_handler.monoRight->getResolutionWidth(), oak_handler.monoRight->getResolutionHeight());

  double fx = oak_handler.intrinsics[0][0]; double cx = oak_handler.intrinsics[0][2];
  double fy = oak_handler.intrinsics[1][1]; double cy = oak_handler.intrinsics[1][2];
  // for(auto row : oak_handler.intrinsics) {
  //     for(auto val : row) cout << val << "  ";
  //     cout << endl;
  // }


  while (ros::ok()) {
    if(oak_handler.initialized){
      if(!oak_handler.first){
        ////////// auto can be used
        oak_handler.leftQueue = device.getOutputQueue("left", 5, false);
        oak_handler.rightQueue = device.getOutputQueue("right", 5, false);
        oak_handler.DepthQueue = device.getOutputQueue("depth", 5, false);
        oak_handler.DisparityQueue = device.getOutputQueue("disparity", 5, false);
        oak_handler.rgbQueue = device.getOutputQueue("rgb", 5, false);
        oak_handler.nNetDataQueue = device.getOutputQueue("detections", 5, false);
        oak_handler.nNetImgQueue = device.getOutputQueue("detected_img", 5, false);
        oak_handler.first = true;
      }
      else{

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        if (oak_handler.get_rgb){
          vector<std::shared_ptr<dai::ImgFrame>> inPassRgb = oak_handler.rgbQueue->tryGetAll<dai::ImgFrame>();
          if (inPassRgb.size()>0){
            FrameRgb = inPassRgb.back()->getCvFrame(); // important
            // cv::imshow("rgb", FrameRgb);
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
        }

        if (oak_handler.get_stereo_ir){
          vector<std::shared_ptr<dai::ImgFrame>> inPassLeft = oak_handler.leftQueue->tryGetAll<dai::ImgFrame>();
          vector<std::shared_ptr<dai::ImgFrame>> inPassRight = oak_handler.rightQueue->tryGetAll<dai::ImgFrame>();
          if (inPassLeft.size()>0){
            FrameLeft = inPassLeft.back()->getFrame();
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
          if (inPassRight.size()>0){
            FrameRight = inPassRight.back()->getFrame();
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
        }

        if (oak_handler.get_stereo_depth or oak_handler.get_pointcloud){
          vector<std::shared_ptr<dai::ImgFrame>> inPassDepth = oak_handler.DepthQueue->tryGetAll<dai::ImgFrame>();
          if (inPassDepth.size()>0){
            FrameDepth = inPassDepth.back()->getFrame();
            if (oak_handler.get_stereo_depth){
              cv_bridge::CvImage bridge_depth = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, FrameDepth);
              bridge_depth.toImageMsg(oak_handler.depth_img_msg);
              oak_handler.d_pub.publish(oak_handler.depth_img_msg);
            }
            if (oak_handler.get_pointcloud){
              pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl;
              for (int i = 0; i < FrameDepth.rows; ++i){
                for (int j = 0; j < FrameDepth.cols; ++j){
                  float temp_depth = FrameDepth.at<ushort>(i,j);
                  if (temp_depth/1000.0 >= 0.1 and temp_depth/1000.0 <= oak_handler.pcl_max_range){
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
        }

        if (oak_handler.get_stereo_disparity){
          vector<std::shared_ptr<dai::ImgFrame>> inPassDisparity = oak_handler.DisparityQueue->tryGetAll<dai::ImgFrame>();
          if (inPassDisparity.size()>0){
            FrameDepthColor = inPassDisparity.back()->getFrame();
            FrameDepthColor.convertTo(FrameDepthColor, CV_8UC1, 255.0 / oak_handler.stereodepth->getMaxDisparity());
            cv::applyColorMap(FrameDepthColor, FrameDepthColor, cv::COLORMAP_JET);
            cv_bridge::CvImage bridge_disparity = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, FrameDepthColor);
            bridge_disparity.toImageMsg(oak_handler.disparity_img_msg);
            oak_handler.dis_pub.publish(oak_handler.disparity_img_msg);
          }
        }


        if (oak_handler.get_YOLO){
          vector<std::shared_ptr<dai::ImgDetections>> inPassNN = oak_handler.nNetDataQueue->tryGetAll<dai::ImgDetections>();
          vector<std::shared_ptr<dai::ImgFrame>> inPassNN_img = oak_handler.nNetImgQueue->tryGetAll<dai::ImgFrame>();
          oakd_ros::bboxes bboxes_msg;
          if (inPassNN_img.size()>0){
            FrameDetect = inPassNN_img.back()->getCvFrame();
            if (inPassNN.size()>0){
              std::vector<dai::ImgDetection> detections = inPassNN.back()->detections;
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
        }
      }
    }
  }
  return 0;
}