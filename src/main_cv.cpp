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
  oak_handler.intrinsics=calibData.getCameraIntrinsics(dai::CameraBoardSocket::RGB, oak_handler.camRgb->getIspWidth(), oak_handler.camRgb->getIspHeight());

  //fx oak_handler.intrinsics[0][0] 
  //cx oak_handler.intrinsics[0][2]
  //fy oak_handler.intrinsics[1][1]
  //cy oak_handler.intrinsics[1][2]
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
        // get_pointcloud, get_raw, get_compressed;
        if (oak_handler.get_rgb){
          vector<std::shared_ptr<dai::ImgFrame>> inPassRgb = oak_handler.rgbQueue->tryGetAll<dai::ImgFrame>();
          if (inPassRgb.size()>0){
            FrameRgb = inPassRgb.back()->getCvFrame(); // important
            cv::imshow("rgb", FrameRgb);
          }
        }
        if (oak_handler.get_stereo_ir){
          vector<std::shared_ptr<dai::ImgFrame>> inPassLeft = oak_handler.leftQueue->tryGetAll<dai::ImgFrame>();
          vector<std::shared_ptr<dai::ImgFrame>> inPassRight = oak_handler.rightQueue->tryGetAll<dai::ImgFrame>();
          if (inPassLeft.size()>0){
            FrameLeft = inPassLeft.back()->getFrame();
            cv::imshow("left", FrameLeft);
          }
          if (inPassRight.size()>0){
            FrameRight = inPassRight.back()->getFrame();
            cv::imshow("right", FrameRight);
          }
        }
        if (oak_handler.get_stereo_depth){
          vector<std::shared_ptr<dai::ImgFrame>> inPassDepth = oak_handler.DepthQueue->tryGetAll<dai::ImgFrame>();
          if (inPassDepth.size()>0){
            FrameDepth = inPassDepth.back()->getFrame();
            cv::imshow("depth", FrameDepth);
          }
        }
        if (oak_handler.get_stereo_disparity){
          vector<std::shared_ptr<dai::ImgFrame>> inPassDisparity = oak_handler.DisparityQueue->tryGetAll<dai::ImgFrame>();
          if (inPassDisparity.size()>0){
            FrameDepthColor = inPassDisparity.back()->getFrame();
            FrameDepthColor.convertTo(FrameDepthColor, CV_8UC1, 255.0 / oak_handler.stereodepth->getMaxDisparity());
            cv::applyColorMap(FrameDepthColor, FrameDepthColor, cv::COLORMAP_JET);
            cv::imshow("disparity_color", FrameDepthColor);
          }
        }
        if (oak_handler.get_YOLO){
          vector<std::shared_ptr<dai::ImgDetections>> inPassNN = oak_handler.nNetDataQueue->tryGetAll<dai::ImgDetections>();
          vector<std::shared_ptr<dai::ImgFrame>> inPassNN_img = oak_handler.nNetImgQueue->tryGetAll<dai::ImgFrame>();
          if (inPassNN_img.size()>0){
            FrameDetect = inPassNN_img.back()->getCvFrame();
            if (inPassNN.size()>0){
              std::vector<dai::ImgDetection> detections = inPassNN.back()->detections;
              for(auto& detection : detections) {
                  int x1 = detection.xmin * FrameDetect.cols;
                  int y1 = detection.ymin * FrameDetect.rows;
                  int x2 = detection.xmax * FrameDetect.cols;
                  int y2 = detection.ymax * FrameDetect.rows;

                  int labelIndex = detection.label;
                  std::string labelStr = to_string(labelIndex);
                  if(labelIndex < oak_handler.class_names.size()) {
                      labelStr = oak_handler.class_names[labelIndex];
                  }
                  cv::putText(FrameDetect, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
                  std::stringstream confStr;
                  confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
                  cv::putText(FrameDetect, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
                  cv::rectangle(FrameDetect, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
              }
            }
            // Show the FrameDetect
            cv::imshow("detection", FrameDetect);
          }
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
          exit(1);
        }
      }
    }
  }
  return 0;
}