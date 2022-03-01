#ifndef OAKD_MAIN_H
#define OAKD_MAIN_H

///// depthai - OAK-D
#include <depthai/depthai.hpp>

///// common headers
#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>

///// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <oakd_ros/bboxes.h>

///// image processing
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

///// pointcloud
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

///// utils
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

using namespace std;

sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link")
{
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  cloud_ROS.header.stamp = ros::Time::now();
  return cloud_ROS;
}



////////////////////////////////////////////////////////////////////////////////////////////////////
class oakd_ros_class{
  public:
    dai::Pipeline pipeline;

    string path;

    /// messages for publishing
    sensor_msgs::Image l_img_msg, r_img_msg, rgb_img_msg, depth_img_msg, nn_img_msg;
    sensor_msgs::CompressedImage l_img_comp_msg, r_img_comp_msg, rgb_img_comp_msg, nn_img_comp_msg;

    bool initialized=false;
    bool get_imu, get_stereo_ir, get_rgb, get_stereo_depth, get_YOLO, get_pointcloud, get_raw, get_compressed;

    string topic_prefix, blob_file, class_file;
    int fps_IMU, infer_img_width, infer_img_height, class_num, thread_num, bilateral_sigma, depth_confidence;
    double fps_rgb_yolo, fps_stereo_depth, confidence_threshold, iou_threshold, pcl_max_range, pcl_min_range;
    vector<string> class_names;

    // for PCL, calib data
    vector<vector<float>> intrinsics;
    int depth_width, depth_height;
 
    // Depth post processing
    bool use_spatialFilter, use_temporalFilter, use_speckleFilter;
    float spatialFilter_alpha, temporalFilter_alpha;
    int spatialFilter_delta, temporalFilter_delta;
    int spatialFilter_holefilling_radius, spatialFilter_iteration_num, speckleFilter_range;

    // PRO version IR and LED
    float IR_laser_brightness_mA, LED_illuminator_brightness_mA;

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Publisher imu_pub, l_pub, l_comp_pub, r_pub, r_comp_pub, rgb_pub, rgb_comp_pub, d_pub, pcl_pub, nn_pub, nn_comp_pub, nn_bbox_pub;
    void main_initialize();



    oakd_ros_class(ros::NodeHandle& n) : nh(n){
      ///// params
      nh.param<std::string>("/topic_prefix", topic_prefix, "/oakd");
      nh.param("/fps_rgb_yolo", fps_rgb_yolo, 30.0);
      nh.param("/fps_stereo_depth", fps_stereo_depth, 30.0);
      nh.param("/fps_IMU", fps_IMU, 200);

      nh.param<bool>("/get_raw", get_raw, false);
      nh.param<bool>("/get_compressed", get_compressed, false);
      
      nh.param<bool>("/get_imu", get_imu, false);
      nh.param<bool>("/get_rgb", get_rgb, false);
      nh.param<bool>("/get_stereo_ir", get_stereo_ir, false);
      nh.param<bool>("/get_stereo_depth", get_stereo_depth, false);
      nh.param<bool>("/get_pointcloud", get_pointcloud, false);
      nh.param<bool>("/get_YOLO", get_YOLO, false);

      nh.param("/pcl_max_range", pcl_max_range, 6.0);
      nh.param("/pcl_min_range", pcl_min_range, 0.3);
      nh.param("/thread_num", thread_num, 3);
      nh.param("/bilateral_sigma", bilateral_sigma, 500);
      
      nh.param("/depth_confidence", depth_confidence, 200);
      nh.param<bool>("/use_spatialFilter", use_spatialFilter, false);
      nh.param<bool>("/use_temporalFilter", use_temporalFilter, false);
      nh.param<bool>("/use_speckleFilter", use_speckleFilter, false);
      nh.param("/spatialFilter_holefilling_radius", spatialFilter_holefilling_radius, 2);
      nh.param("/spatialFilter_iteration_num", spatialFilter_iteration_num, 1);
      nh.param<float>("/spatialFilter_alpha", spatialFilter_alpha, 0.5);
      nh.param("/spatialFilter_delta", spatialFilter_delta, 20);
      nh.param<float>("/temporalFilter_alpha", temporalFilter_alpha, 0.4);
      nh.param("/temporalFilter_delta", temporalFilter_delta, 20);
      nh.param("/speckleFilter_range", speckleFilter_range, 50);
      
      nh.param<std::string>("/blob_file", blob_file, "/blob_files/tiny-yolo-v4.blob");
      nh.param<std::string>("/class_file", class_file, "/blob_files/class.txt");
      nh.param("/infer_img_width", infer_img_width, 416);
      nh.param("/infer_img_height", infer_img_height, 416);
      nh.param("/class_num", class_num, 80);
      nh.param("/confidence_threshold", confidence_threshold, 0.7);
      nh.param("/iou_threshold", iou_threshold, 0.7);

      nh.param<float>("/IR_laser_brightness_mA", IR_laser_brightness_mA, 0.0);
      nh.param<float>("/LED_illuminator_brightness_mA", LED_illuminator_brightness_mA, 0.0);


      ///// sub pub
      if (get_rgb){
        if (get_raw)
          rgb_pub = nh.advertise<sensor_msgs::Image>(topic_prefix+"/rgb/image_raw", 10);
        if (get_compressed)
          rgb_comp_pub = nh.advertise<sensor_msgs::CompressedImage>(topic_prefix+"/rgb/image_raw/compressed", 10);
      }
      if (get_stereo_ir){
        if (get_raw){
          l_pub = nh.advertise<sensor_msgs::Image>(topic_prefix+"/stereo_ir/left/image_raw", 10);
          r_pub = nh.advertise<sensor_msgs::Image>(topic_prefix+"/stereo_ir/right/image_raw", 10);
        }
        if (get_compressed){
          l_comp_pub = nh.advertise<sensor_msgs::CompressedImage>(topic_prefix+"/stereo_ir/left/image_raw/compressed", 10);
          r_comp_pub = nh.advertise<sensor_msgs::CompressedImage>(topic_prefix+"/stereo_ir/right/image_raw/compressed", 10);
        }
      }
      if (get_stereo_depth)
        d_pub = nh.advertise<sensor_msgs::Image>(topic_prefix+"/depth/image_raw", 10);
      if (get_pointcloud)
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_prefix+"/pointcloud", 10);
      if (get_YOLO){
        nn_bbox_pub = nh.advertise<oakd_ros::bboxes>(topic_prefix+"/yolo/bboxes", 10);
        if (get_raw)
          nn_pub = nh.advertise<sensor_msgs::Image>(topic_prefix+"/yolo/image_raw", 10);
        if (get_compressed)
          nn_comp_pub = nh.advertise<sensor_msgs::CompressedImage>(topic_prefix+"/yolo/image_raw/compressed", 10);
      }
      if (get_imu)
        imu_pub = nh.advertise<sensor_msgs::Imu>(topic_prefix+"/imu",10);


      ///// Init
      path = ros::package::getPath("oakd_ros");
      main_initialize();
      ROS_WARN("class heritated, starting node...");
    }
};




//////////// can be separated into .cpp source file
void oakd_ros_class::main_initialize(){
  if (get_imu){
    std::shared_ptr<dai::node::IMU> IMU_node = pipeline.create<dai::node::IMU>();
    std::shared_ptr<dai::node::XLinkOut> xoutIMU = pipeline.create<dai::node::XLinkOut>();
    xoutIMU->setStreamName("imu");

    IMU_node->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW, dai::IMUSensor::ROTATION_VECTOR}, fps_IMU);
    IMU_node->setBatchReportThreshold(1);
    IMU_node->setMaxBatchReports(28);
    IMU_node->out.link(xoutIMU->input);
  }
  if(get_rgb){
    std::shared_ptr<dai::node::ColorCamera> camRgb = pipeline.create<dai::node::ColorCamera>();
    std::shared_ptr<dai::node::XLinkOut> xoutRgb = pipeline.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");

    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setFps(fps_rgb_yolo);
    // camRgb->initialControl.setManualFocus(135);
    camRgb->setPreviewSize(640, 400);
    camRgb->setInterleaved(false);
    camRgb->preview.link(xoutRgb->input);
    
    if(get_YOLO){
      std::shared_ptr<dai::node::ImageManip> Manipulator  = pipeline.create<dai::node::ImageManip>();
      std::shared_ptr<dai::node::YoloDetectionNetwork> detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();
      std::shared_ptr<dai::node::XLinkOut> xoutInference  = pipeline.create<dai::node::XLinkOut>();
      std::shared_ptr<dai::node::XLinkOut> nnOut = pipeline.create<dai::node::XLinkOut>();
      nnOut->setStreamName("detections");
      xoutInference->setStreamName("detected_img");

      camRgb->preview.link(Manipulator->inputImage);
      Manipulator->initialConfig.setResizeThumbnail(infer_img_width, infer_img_height);
      Manipulator->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
      Manipulator->inputImage.setBlocking(false);
      Manipulator->out.link(xoutInference->input);
      Manipulator->out.link(detectionNetwork->input);
      detectionNetwork->setBlobPath(path+blob_file);
      detectionNetwork->setNumInferenceThreads(thread_num);
      detectionNetwork->setConfidenceThreshold(confidence_threshold);
      detectionNetwork->setIouThreshold(iou_threshold);
      detectionNetwork->setNumClasses(class_num);
      detectionNetwork->input.setBlocking(false);
      detectionNetwork->setCoordinateSize(4);
      detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
      detectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
      detectionNetwork->out.link(nnOut->input);
      // detectionNetwork->passthrough.link(xoutInference->input);
      ifstream readfile;
      readfile.open(path+class_file);
      while (!readfile.eof()){
        string str;
        getline(readfile, str);
        class_names.push_back(str);
      }
      readfile.close();
    }
  }
  if (!get_rgb && get_YOLO){
    std::shared_ptr<dai::node::ColorCamera> camRgb = pipeline.create<dai::node::ColorCamera>();
    std::shared_ptr<dai::node::YoloDetectionNetwork> detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();
    std::shared_ptr<dai::node::XLinkOut> xoutInference  = pipeline.create<dai::node::XLinkOut>();
    std::shared_ptr<dai::node::XLinkOut> nnOut = pipeline.create<dai::node::XLinkOut>();
    nnOut->setStreamName("detections");
    xoutInference->setStreamName("detected_img");

    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setFps(fps_rgb_yolo);
    // camRgb->initialControl.setManualFocus(135);
    camRgb->setInterleaved(false);
    camRgb->setPreviewSize(infer_img_width, infer_img_height);
    camRgb->preview.link(detectionNetwork->input);
    camRgb->preview.link(xoutInference->input);

    detectionNetwork->setBlobPath(path+blob_file);
    detectionNetwork->setNumInferenceThreads(thread_num);
    detectionNetwork->setConfidenceThreshold(confidence_threshold);
    detectionNetwork->setIouThreshold(iou_threshold);
    detectionNetwork->setNumClasses(class_num);
    detectionNetwork->input.setBlocking(false);
    detectionNetwork->setCoordinateSize(4);
    detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    detectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    detectionNetwork->out.link(nnOut->input);
    // detectionNetwork->passthrough.link(xoutInference->input);
    ifstream readfile;
    readfile.open(path+class_file);
    while (!readfile.eof()){
      string str;
      getline(readfile, str);
      class_names.push_back(str);
    }
    readfile.close();
  }

  if(get_stereo_ir){
    std::shared_ptr<dai::node::MonoCamera> monoLeft     = pipeline.create<dai::node::MonoCamera>();
    std::shared_ptr<dai::node::MonoCamera> monoRight    = pipeline.create<dai::node::MonoCamera>();
    std::shared_ptr<dai::node::XLinkOut> xoutLeft       = pipeline.create<dai::node::XLinkOut>();
    std::shared_ptr<dai::node::XLinkOut> xoutRight      = pipeline.create<dai::node::XLinkOut>();
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(fps_stereo_depth);
    // monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(fps_stereo_depth);

    monoLeft->out.link(xoutLeft->input);
    monoRight->out.link(xoutRight->input);

    if (get_stereo_depth || get_pointcloud){
      std::shared_ptr<dai::node::StereoDepth> stereodepth = pipeline.create<dai::node::StereoDepth>();
      std::shared_ptr<dai::node::XLinkOut> xoutDepth      = pipeline.create<dai::node::XLinkOut>();
      xoutDepth->setStreamName("depth");

      stereodepth->initialConfig.setConfidenceThreshold(depth_confidence);
      stereodepth->setLeftRightCheck(true);
      stereodepth->initialConfig.setLeftRightCheckThreshold(10);
      stereodepth->initialConfig.setBilateralFilterSigma(bilateral_sigma);
      stereodepth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
      stereodepth->setDepthAlign(dai::CameraBoardSocket::LEFT); //default: Right
      // stereodepth->setRectifyEdgeFillColor(0); // black, to better see the cutout
      stereodepth->setExtendedDisparity(false);
      stereodepth->setSubpixel(true);

      dai::RawStereoDepthConfig depth_config = stereodepth->initialConfig.get();
      if (use_spatialFilter){
        ROS_WARN("SPATIAL FILTER");
        depth_config.postProcessing.spatialFilter.enable = true;
        depth_config.postProcessing.spatialFilter.holeFillingRadius = spatialFilter_holefilling_radius;
        depth_config.postProcessing.spatialFilter.numIterations = spatialFilter_iteration_num;
        depth_config.postProcessing.spatialFilter.alpha = spatialFilter_alpha;
        depth_config.postProcessing.spatialFilter.delta = spatialFilter_delta;
      }
      if (use_temporalFilter){
        ROS_WARN("TEMPORAL FILTER");
        depth_config.postProcessing.temporalFilter.enable = true;
        depth_config.postProcessing.temporalFilter.alpha = temporalFilter_alpha;
        depth_config.postProcessing.temporalFilter.delta = temporalFilter_delta;
        depth_config.postProcessing.temporalFilter.persistencyMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_4;
        // depth_config.postProcessing.temporalFilter.persistencyMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_8_OUT_OF_8;
      }
      if (use_speckleFilter){
        ROS_WARN("SPECKLE FILTER");
        depth_config.postProcessing.speckleFilter.enable = true;
        depth_config.postProcessing.speckleFilter.speckleRange = speckleFilter_range;
      }
      stereodepth->initialConfig.set(depth_config);

      monoLeft->out.link(stereodepth->left);
      monoRight->out.link(stereodepth->right);

      // stereodepth->syncedLeft.link(xoutLeft->input);
      // stereodepth->syncedRight.link(xoutRight->input);
      stereodepth->left.setBlocking(false);
      stereodepth->right.setBlocking(false);
      stereodepth->left.setQueueSize(1);
      stereodepth->right.setQueueSize(1);
      stereodepth->depth.link(xoutDepth->input);
      
      depth_width = monoRight->getResolutionWidth();
      depth_height= monoRight->getResolutionHeight();
    }
  }
  if(!get_stereo_ir && (get_stereo_depth || get_pointcloud)){
    std::shared_ptr<dai::node::MonoCamera> monoLeft     = pipeline.create<dai::node::MonoCamera>();
    std::shared_ptr<dai::node::MonoCamera> monoRight    = pipeline.create<dai::node::MonoCamera>();
    std::shared_ptr<dai::node::StereoDepth> stereodepth = pipeline.create<dai::node::StereoDepth>();
    std::shared_ptr<dai::node::XLinkOut> xoutDepth      = pipeline.create<dai::node::XLinkOut>();
    xoutDepth->setStreamName("depth");

    // monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(fps_stereo_depth);
    // monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(fps_stereo_depth);

    stereodepth->initialConfig.setConfidenceThreshold(depth_confidence);
    stereodepth->setLeftRightCheck(true);
    stereodepth->initialConfig.setLeftRightCheckThreshold(10);
    stereodepth->initialConfig.setBilateralFilterSigma(bilateral_sigma);
    stereodepth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereodepth->setDepthAlign(dai::CameraBoardSocket::LEFT); //default: Right
    // stereodepth->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereodepth->setExtendedDisparity(false);
    stereodepth->setSubpixel(true);

    dai::RawStereoDepthConfig depth_config = stereodepth->initialConfig.get();
    if (use_spatialFilter){
      ROS_WARN("SPATIAL FILTER");
      depth_config.postProcessing.spatialFilter.enable = true;
      depth_config.postProcessing.spatialFilter.holeFillingRadius = spatialFilter_holefilling_radius;
      depth_config.postProcessing.spatialFilter.numIterations = spatialFilter_iteration_num;
      depth_config.postProcessing.spatialFilter.alpha = spatialFilter_alpha;
      depth_config.postProcessing.spatialFilter.alpha = spatialFilter_alpha;
    }
    if (use_temporalFilter){
      ROS_WARN("TEMPORAL FILTER");
      depth_config.postProcessing.temporalFilter.enable = true;
      depth_config.postProcessing.temporalFilter.alpha = temporalFilter_alpha;
      depth_config.postProcessing.temporalFilter.delta = temporalFilter_delta;
      depth_config.postProcessing.temporalFilter.persistencyMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_4;
      // depth_config.postProcessing.temporalFilter.persistencyMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_8_OUT_OF_8;
    }
    if (use_speckleFilter){
      ROS_WARN("SPECKLE FILTER");
      depth_config.postProcessing.speckleFilter.enable = true;
      depth_config.postProcessing.speckleFilter.speckleRange = speckleFilter_range;
    }
    stereodepth->initialConfig.set(depth_config);

    monoLeft->out.link(stereodepth->left);
    monoRight->out.link(stereodepth->right);

    stereodepth->left.setBlocking(false);
    stereodepth->right.setBlocking(false);
    stereodepth->left.setQueueSize(1);
    stereodepth->right.setQueueSize(1);
    stereodepth->depth.link(xoutDepth->input);
    
    depth_width = monoRight->getResolutionWidth();
    depth_height= monoRight->getResolutionHeight();
  }

  initialized=true;
}



#endif