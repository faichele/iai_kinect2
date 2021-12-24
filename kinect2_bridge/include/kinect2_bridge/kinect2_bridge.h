#ifndef KINECT2_BRIDGE_H
#define KINECT2_BRIDGE_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#if defined(__linux__)
#include <sys/prctl.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

#if defined(__linux__)
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>
#else
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

// Windows Header Files
#include <windows.h>
// Kinect Header files
#include <Kinect.h>
#endif

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>

#include <compressed_depth_image_transport/compression_common.h>

#include <kinect2_bridge/kinect2_definitions.h>
#include <kinect2_registration/kinect2_registration.h>
#include <kinect2_registration/kinect2_console.h>
#include <cv_bridge/cv_bridge.h>
#include <bb_kinect2_msgs/CameraIntrinsics.h>

#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/circular_buffer.hpp>

class Kinect2BridgePrivate;
class Kinect2Bridge
{
public:
    Kinect2Bridge(bool readImages, const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &priv_nh = ros::NodeHandle("~"));
    virtual ~Kinect2Bridge();

    bool start();
    void stop();

    bool receiveFrames();

    enum ImageDataSource
    {
        IMAGE_DATA_SOURCE_RGB = 0,
        IMAGE_DATA_SOURCE_IR = 1,
        IMAGE_DATA_SOURCE_DEPTH = 2
    };

    enum Image
    {
        IR_SD = 0,
        IR_SD_RECT,

        DEPTH_SD,
        DEPTH_SD_RECT,
        DEPTH_HD,
        DEPTH_QHD,

        COLOR_SD_RECT,
        COLOR_HD,
        COLOR_HD_RECT,
        COLOR_QHD,
        COLOR_QHD_RECT,

        MONO_HD,
        MONO_HD_RECT,
        MONO_QHD,
        MONO_QHD_RECT,

        COUNT
    };

    enum Status
    {
        UNSUBCRIBED = 0,
        RAW,
        COMPRESSED,
        BOTH
    };

    struct ImagePublisherOption
    {
    public:
            bool valid;
            Image imageType;
            bool publishCompressed;
            bool publish;

            ImagePublisherOption() : imageType(COUNT), publish(false), publishCompressed(false), valid(false)
            {

            }

            ImagePublisherOption(const ImagePublisherOption& other)
            {
                if (this != &other)
                {
                    imageType = other.imageType;
                    publish = other.publish;
                    publishCompressed = other.publishCompressed;
                    valid = other.valid;
                }
            }

            ImagePublisherOption& operator=(const ImagePublisherOption& other)
            {
                if (this != &other)
                {
                    imageType = other.imageType;
                    publish = other.publish;
                    publishCompressed = other.publishCompressed;
                    valid = other.valid;
                }

                return *this;
            }
    };

private:
    bool initialize();
    bool initRegistration(const std::string &method, const int32_t device, const double maxDepth);
    bool initPipeline(const std::string &method, const int32_t device);
    void initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth);
    void initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png);
    void initTopics(const int32_t queueSize, const std::string &base_name);
    bool initDevice(const std::string &sensor);
    void initCalibration(const std::string &calib_path, const std::string &sensor);

    bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const;
    bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const;
    bool loadCalibrationDepthFile(const std::string &filename, double &depthShift) const;

    void createCameraInfo();
    void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const;

    void callbackStatus();
    bool updateStatus(bool &isSubscribedColor, bool &isSubscribedDepth);

    void main();

    void threadDispatcher(const size_t id);

    void receiveIrDepth();
    void receiveColor();

    std_msgs::Header createHeader(ros::Time &last, ros::Time &other);

    void processColor(std::vector<cv::Mat> &images, const std::vector<Status> &status);
    void processIrDepth(const cv::Mat &depth, std::vector<cv::Mat> &images, const std::vector<Kinect2Bridge::Status> &status);

    void publishImages(const std::vector<cv::Mat> &images, const std_msgs::Header &header, const std::vector<Status> &status, const size_t frame, size_t &pubFrame, const size_t begin, const size_t end);
    void publishStaticTF();

    void createImage(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::Image &msgImage) const;
    void createCompressed(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::CompressedImage &msgImage) const;

    static inline void setThreadName(const std::string &name)
    {
  #if defined(__linux__)
      prctl(PR_SET_NAME, name.c_str());
  #elif defined(__APPLE__)
      pthread_setname_np(name.c_str());
  #endif
    }

    bool m_readImages;

    boost::shared_ptr<Kinect2BridgePrivate> m_d;

    std::map<int, std::string> keypoint_indices_to_joint_names_map;
    std::map<std::string, std::pair<int, int>> bone_name_to_keypoint_indices_map;

    std::string kinect_sensor;

    std::vector<int> compressionParams;
    std::string compression16BitExt, compression16BitString, baseNameTF;

    cv::Size sizeColor, sizeIr, sizeLowRes;


    cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;

    std::vector<std::thread> threads;
    std::mutex lockIrDepth, lockColor;
    std::mutex lockSync, lockPub, lockTime, lockStatus;
    std::mutex lockRegLowRes, lockRegHighRes, lockRegSD;

    bool publishTF;
    std::thread tfPublisher, mainThread;

    ros::NodeHandle nh, priv_nh;

    DepthRegistration *depthRegLowRes, *depthRegHighRes;

    size_t frameColor, frameIrDepth, pubFrameColor, pubFrameIrDepth;
    ros::Time lastColor, lastDepth;

    bool nextColor, nextIrDepth;
    double deltaT, depthShift, elapsedTimeColor, elapsedTimeIrDepth;
    bool running, deviceActive, clientConnected, isSubscribedColor, isSubscribedDepth;

    std::vector<std::string> imageTopics;
    std::vector<bool> publishCompressedImages;
    std::map<Image, ros::Publisher> imagePubs, compressedPubs;
    ros::Publisher infoHDPub, infoQHDPub, infoIRPub;
    sensor_msgs::CameraInfo infoHD, infoQHD, infoIR;
    std::vector<Status> status;

    Image stringToImageType(const std::string&);
    bool retrieveImagePubOptions();
    const ImagePublisherOption& getImagePublisherOption(const Image);

    bool retrieveColorAndIrParams();

    static ImagePublisherOption emptyImagePublisherOption;
    bool imagePubOptionsRetrieved;
    std::vector<ImagePublisherOption> imagePublisherOptions;
};

class Kinect2BridgeNodelet : public nodelet::Nodelet
{
private:
  Kinect2Bridge *pKinect2Bridge;

public:
  Kinect2BridgeNodelet();

  ~Kinect2BridgeNodelet();

  virtual void onInit();
};

#endif
