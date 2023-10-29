#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

std::shared_ptr<dai::Pipeline> createPipeline() {
    // Start defining a pipeline
    auto pipeline = std::make_shared<dai::Pipeline>();
    // Define a sources
    auto camRgb = pipeline->create<dai::node::ColorCamera>();
    auto stereoDepth = pipeline->create<dai::node::StereoDepth>();
    auto monoLeft = pipeline->create<dai::node::MonoCamera>();
    auto monoRight = pipeline->create<dai::node::MonoCamera>();

    camRgb->setPreviewSize(640, 400);
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    stereoDepth->initialConfig.setConfidenceThreshold(190);
    stereoDepth->setLeftRightCheck(true);
    stereoDepth->initialConfig.setLeftRightCheckThreshold(10);
    stereoDepth->initialConfig.setBilateralFilterSigma(5);
    stereoDepth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereoDepth->setDepthAlign(dai::CameraBoardSocket::LEFT); //default: Right
    stereoDepth->setExtendedDisparity(false);
    stereoDepth->setSubpixel(true);

    // Depth config
    dai::RawStereoDepthConfig depth_config = stereoDepth->initialConfig.get();
    depth_config.postProcessing.spatialFilter.enable = true;
    depth_config.postProcessing.spatialFilter.holeFillingRadius = 2;
    depth_config.postProcessing.spatialFilter.numIterations = 1;
    depth_config.postProcessing.spatialFilter.alpha = 0.5;
    depth_config.postProcessing.spatialFilter.delta = 20;
    depth_config.postProcessing.temporalFilter.enable = true;
    depth_config.postProcessing.temporalFilter.alpha = 0.4;
    depth_config.postProcessing.temporalFilter.delta = 20;
    depth_config.postProcessing.temporalFilter.persistencyMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_4;
    // depth_config.postProcessing.temporalFilter.persistencyMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_8_OUT_OF_8;
    depth_config.postProcessing.speckleFilter.enable = true;
    depth_config.postProcessing.speckleFilter.speckleRange = 200;
    stereoDepth->initialConfig.set(depth_config);

    // Depth needs left and right links
    monoLeft->out.link(stereoDepth->left);
    monoRight->out.link(stereoDepth->right);
    
    // Create output
    auto xoutRgb = pipeline->create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline->create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    xoutDepth->setStreamName("depth");
    camRgb->preview.link(xoutRgb->input);
    stereoDepth->depth.link(xoutDepth->input);

    return pipeline;
}

int main(int argc, char** argv) {
    auto deviceInfoVec = dai::Device::getAllAvailableDevices();
    const auto usbSpeed = dai::UsbSpeed::SUPER;
    auto openVinoVersion = dai::OpenVINO::Version::VERSION_2021_4;

    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> qRgbMap;
    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> qDepthMap;
    std::vector<std::shared_ptr<dai::Device>> devices;

    for(auto& deviceInfo : deviceInfoVec) {
        auto device = std::make_shared<dai::Device>(openVinoVersion, deviceInfo, usbSpeed);
        devices.push_back(device);
        std::cout << "===Connected to " << deviceInfo.getMxId() << std::endl;
        auto mxId = device->getMxId();
        auto cameras = device->getConnectedCameras();
        auto usbSpeed = device->getUsbSpeed();
        auto eepromData = device->readCalibration2().getEepromData();
        std::cout << "   >>> MXID:" << mxId << std::endl;
        std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
        std::cout << "   >>> USB speed:" << usbSpeed << std::endl;
        if(eepromData.boardName != "") {
            std::cout << "   >>> Board name:" << eepromData.boardName << std::endl;
        }
        if(eepromData.productName != "") {
            std::cout << "   >>> Product name:" << eepromData.productName << std::endl;
        }
        auto pipeline = createPipeline();
        device->startPipeline(*pipeline);

        auto qRgb = device->getOutputQueue("rgb", 4, false);
        auto qDepth = device->getOutputQueue("depth", 4, false);
        std::string streamName = "rgb-" + eepromData.productName + mxId;
        std::string streamName2 = "depth-" + eepromData.productName + mxId;
        qRgbMap.insert({streamName, qRgb});
        qDepthMap.insert({streamName2, qDepth});
    }
    while(true) {
        for(auto& element : qRgbMap) {
            auto qRgb = element.second;
            auto streamName = element.first;
            auto inRgb = qRgb->tryGet<dai::ImgFrame>();
            if(inRgb != nullptr) {
                cv::imshow(streamName, inRgb->getCvFrame());
            }
        }
        for(auto& element : qDepthMap) {
            auto qDepth = element.second;
            auto streamName = element.first;
            auto inDepth = qDepth->tryGet<dai::ImgFrame>();
            if(inDepth != nullptr) {
                cv::Mat depthImgFloat = inDepth->getCvFrame();
                double min_, max_;
                cv::minMaxIdx(depthImgFloat, &min_, &max_);
                cv::Mat coloredDepthImg, depthImg;
                depthImgFloat.convertTo(depthImg, CV_8UC1, 255.0 / (max_ - min_), -min_);
                cv::applyColorMap(depthImg, coloredDepthImg, cv::COLORMAP_JET);
                cv::imshow(streamName, coloredDepthImg);
            }
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}