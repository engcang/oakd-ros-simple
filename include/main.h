#ifndef OAKD_MAIN_H
#define OAKD_MAIN_H

///// depthai - OAK-D
#include <depthai/depthai.hpp>

///// common headers
#include <string>
#include <vector>

///// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

///// image processing
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

///// utils
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

using namespace std;


////////////////////////////////////////////////////////////////////////////////////////////////////
class oakd_ros{
  public:
    dai::Pipeline pipeline;
    ////////// auto can be used
    std::shared_ptr<dai::node::MonoCamera> monoLeft     = pipeline.create<dai::node::MonoCamera>();
    std::shared_ptr<dai::node::MonoCamera> monoRight    = pipeline.create<dai::node::MonoCamera>();
    std::shared_ptr<dai::node::StereoDepth> stereodepth = pipeline.create<dai::node::StereoDepth>();
    std::shared_ptr<dai::node::ColorCamera> camRgb      = pipeline.create<dai::node::ColorCamera>();
    std::shared_ptr<dai::node::XLinkOut> xoutLeft       = pipeline.create<dai::node::XLinkOut>();
    std::shared_ptr<dai::node::XLinkOut> xoutRight      = pipeline.create<dai::node::XLinkOut>();
    std::shared_ptr<dai::node::XLinkOut> xoutDepth      = pipeline.create<dai::node::XLinkOut>();
    std::shared_ptr<dai::node::XLinkOut> xoutDisparity  = pipeline.create<dai::node::XLinkOut>();
    std::shared_ptr<dai::node::XLinkOut> xoutRgb        = pipeline.create<dai::node::XLinkOut>();
    std::shared_ptr<dai::node::MobileNetDetectionNetwork> detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    std::shared_ptr<dai::node::XLinkOut> nnOut = pipeline.create<dai::node::XLinkOut>();


    std::shared_ptr<dai::DataOutputQueue> leftQueue;
    std::shared_ptr<dai::DataOutputQueue> rightQueue;
    std::shared_ptr<dai::DataOutputQueue> DepthQueue;
    std::shared_ptr<dai::DataOutputQueue> DisparityQueue;
    std::shared_ptr<dai::DataOutputQueue> rgbQueue;
    std::shared_ptr<dai::DataOutputQueue> nNetDataQueue;

    string path;

    sensor_msgs::CompressedImage third_cam_img_msg, first_cam_img_msg;

    bool initialized=false, first=false;
    string first_cam_topic;
    int img_width;
    double rate;

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Publisher l_pub, l_comp_pub, r_pub, r_comp_pub, d_pub, pcl_pub, nn_pub;
    ros::Timer main_timer;

    void main_initialize();

    oakd_ros(ros::NodeHandle& n) : nh(n){

      ///// params
      nh.param("/img_width", img_width, 480);
      nh.param("/rate", rate, 30.0);
      nh.param<std::string>("/first_cam_topic", first_cam_topic, "/d455/depth/rgb_image_raw/compressed");

      ///// sub pub
      pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_topic", 10);

      ///// Init
      path = ros::package::getPath("oakd-ros");
      main_initialize();
      ROS_WARN("class heritated, starting node...");
    }
};

void oakd_ros::main_initialize(){
  xoutLeft->setStreamName("left");
  xoutRight->setStreamName("right");
  xoutDepth->setStreamName("depth");
  xoutDisparity->setStreamName("disparity");
  xoutRgb->setStreamName("rgb");
  nnOut->setStreamName("detections");

  monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
  monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
  monoLeft->setFps(30);
  monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
  monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  monoRight->setFps(30);
  camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
  camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  camRgb->setPreviewSize(416, 416);
  camRgb->setInterleaved(false);
  // camRgb->setIspScale(1, 3);
  camRgb->setFps(30);

  stereodepth->initialConfig.setConfidenceThreshold(160);
  stereodepth->setRectifyEdgeFillColor(0); // black, to better see the cutout
  stereodepth->initialConfig.setLeftRightCheckThreshold(1);
  stereodepth->setLeftRightCheck(true);
  stereodepth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_3x3);
  stereodepth->setExtendedDisparity(false);
  stereodepth->setSubpixel(false);
  stereodepth->setDepthAlign(dai::CameraBoardSocket::RGB); //default: Right

  detectionNetwork->setConfidenceThreshold(0.7f);
  detectionNetwork->setBlobPath(path+"/tiny-yolo-v4.blob");

  // Link plugins CAM -> stereodepth -> XLINK
  monoLeft->out.link(stereodepth->left);
  monoRight->out.link(stereodepth->right);

  stereodepth->syncedLeft.link(xoutLeft->input);
  stereodepth->syncedRight.link(xoutRight->input);
  stereodepth->depth.link(xoutDepth->input);
  stereodepth->disparity.link(xoutDisparity->input);
  camRgb->preview.link(xoutRgb->input);
  camRgb->preview.link(detectionNetwork->input);

  // if(syncNN) detectionNetwork->passthrough.link(xlinkOut->input);
    // else colorCam->preview.link(xlinkOut->input);

  // result of NN out
  detectionNetwork->out.link(nnOut->input);

  initialized=true;
}



#endif