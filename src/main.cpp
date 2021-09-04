#include "main.h"

std::vector<std::string> labelMap = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};
    
auto color = cv::Scalar(255, 0, 0);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "oakd_ros_node");
  ros::NodeHandle n("~");

  oakd_ros oak_handler(n);
  ros::Rate rate(oak_handler.rate);

  signal(SIGINT, signal_handler); // to exit program when ctrl+c


  dai::Device device(oak_handler.pipeline);

  while (ros::ok()) {
    if(oak_handler.initialized){
      if(!oak_handler.first){
        ////////// auto can be used
        oak_handler.leftQueue = device.getOutputQueue("left", 30, false);
        oak_handler.rightQueue = device.getOutputQueue("right", 30, false);
        oak_handler.DepthQueue = device.getOutputQueue("depth", 30, false);
        oak_handler.DisparityQueue = device.getOutputQueue("disparity", 30, false);
        oak_handler.rgbQueue = device.getOutputQueue("rgb", 30, false);
        oak_handler.nNetDataQueue = device.getOutputQueue("detections", 30, false);
        oak_handler.first = true;
      }
      else{
        ////////// auto can be used
        std::shared_ptr<dai::ImgFrame> inPassLeft = oak_handler.leftQueue->get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> inPassRight = oak_handler.rightQueue->get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> inPassDepth = oak_handler.DepthQueue->get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> inPassDisparity = oak_handler.DisparityQueue->get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> inPassRgb = oak_handler.rgbQueue->get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgDetections> inPassNN = oak_handler.nNetDataQueue->get<dai::ImgDetections>();
        std::vector<dai::ImgDetection> detections = inPassNN->detections;


        cv::Mat FrameLeft = inPassLeft->getFrame();
        cv::Mat FrameRight = inPassRight->getFrame();
        cv::Mat FrameDepth = inPassDepth->getFrame();
        cv::Mat FrameRgb = inPassRgb->getFrame();
        cv::Mat FrameDepthColor = inPassDisparity->getFrame();


        cv::imshow("left", FrameLeft);
        cv::imshow("right", FrameRight);
        cv::imshow("depth", FrameDepth);
        cv::imshow("rgb", FrameRgb);

        FrameDepthColor.convertTo(FrameDepthColor, CV_8UC1, 255.0 / oak_handler.stereodepth->getMaxDisparity());
        cv::applyColorMap(FrameDepthColor, FrameDepthColor, cv::COLORMAP_JET);
        cv::imshow("disparity_color", FrameDepthColor);

        for(auto& detection : detections) {
            int x1 = detection.xmin * FrameRgb.cols;
            int y1 = detection.ymin * FrameRgb.rows;
            int x2 = detection.xmax * FrameRgb.cols;
            int y2 = detection.ymax * FrameRgb.rows;

            int labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(FrameRgb, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(FrameRgb, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::rectangle(FrameRgb, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the FrameRgb
        cv::imshow("detection", FrameRgb);


        int key = cv::waitKey(1);
        if(key == 'q') {
          exit(1);
        }
      }
    }
    // rate.sleep();
  }

  return 0;
}