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

#include <iostream>
#include <string>
#include <chrono>

#include <QPixmap>
#include <QtWinExtras/qwinfunctions.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf2/LinearMath/Vector3.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <bb_person_msgs/Person.h>
#include <bb_person_msgs/Persons.h>
#include <bb_person_msgs/TrackingStates.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <kinect2_bridge/freenect_libfreenect2.h>
#include <kinect2_bridge/freenect_frame_listener_impl.h>
#include <kinect2_bridge/freenect_packet_pipeline.h>
#include <kinect2_bridge/freenect_config.h>
#include "kinect2_bridge/freenect_registration.h"

#include <kinect2_bridge/kinect2_definitions.h>
#include <kinect2_registration/kinect2_registration.h>
#include <kinect2_registration/kinect2_console.h>

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface*& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

class Kinect2SkeletonData
{
    public:
        Kinect2SkeletonData(const ros::Time& timeStamp = ros::Time::now())
        {
            this->timeStamp = timeStamp;
        }

        Kinect2SkeletonData(const Kinect2SkeletonData& other)
        {
            if (this != &other)
            {
                timeStamp = other.timeStamp;
                trackingId = other.trackingId;
                trackingIndex = other.trackingIndex;
                for (int k = 0; k < JointType_Count; ++k)
                {
                    joints[k].JointType = other.joints[k].JointType;
                    joints[k].TrackingState = other.joints[k].TrackingState;
                    joints[k].Position.X = other.joints[k].Position.X;
                    joints[k].Position.Y = other.joints[k].Position.Y;
                    joints[k].Position.Z = other.joints[k].Position.Z;

                    jointPositions2D[k] = other.jointPositions2D[k];
                    jointPositions3D[k] = other.jointPositions3D[k];
                }
            }
        }

        Kinect2SkeletonData& operator=(const Kinect2SkeletonData& other)
        {
            if (this != &other)
            {
                timeStamp = other.timeStamp;
                trackingId = other.trackingId;
                trackingIndex = other.trackingIndex;
                for (int k = 0; k < JointType_Count; ++k)
                {
                    joints[k].JointType = other.joints[k].JointType;
                    joints[k].TrackingState = other.joints[k].TrackingState;
                    joints[k].Position.X = other.joints[k].Position.X;
                    joints[k].Position.Y = other.joints[k].Position.Y;
                    joints[k].Position.Z = other.joints[k].Position.Z;

                    jointPositions2D[k] = other.jointPositions2D[k];
                    jointPositions3D[k] = other.jointPositions3D[k];
                }
            }
            return *this;
        }

        ros::Time timeStamp;
        unsigned int trackingId;
        unsigned long trackingIndex;
        Joint joints[JointType_Count];
        tf2::Vector3 jointPositions2D[JointType_Count];
        tf2::Vector3 jointPositions3D[JointType_Count];
};

class Kinect2BridgePrivate
{
public:
    Kinect2BridgePrivate(bool readImages);

    bool readImagesFromDirectory(const std::string& directory = "E:\\Temp");
    void publishNextImageSet();

    std::string cv_mat_type2str(int type);

    bool receiveIrDepth(IMultiSourceFrame*& frame, IInfraredFrame*& pInfraredFrame, IDepthFrame*& pDepthFrame, const std::chrono::time_point<std::chrono::system_clock> timestamp, unsigned int ts_milliseconds);
    bool receiveColor(IMultiSourceFrame*& frame, IColorFrame*& pColorFrame, const std::chrono::time_point<std::chrono::system_clock> timestamp, unsigned int ts_milliseconds);
    bool receiveFrames();

    void processColorDepthFrame();

    bool initDevice(const std::string& sensor);
    bool shutdownDevice();

    bool startDevice(const std::string& sensor);
    bool stopDevice(const std::string& sensor);

    int getNumDevices();
    bool locateDevice(const std::string& sensor);

    bool openDevice(const std::string& sensor);
    bool closeDevice(const std::string& sensor, bool closeOnly = true);

    void createCameraParameters(const std::string& sensor);

    bool initRegistration(const std::string &method, const int32_t device, const double maxDepth);
    bool initPipeline(const std::string &method, const int32_t device);

    void initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth);

    bool startStreams(bool isSubscribedColor, bool isSubscribedDepth);

    bool ProcessDepth(cv::Mat& depth_map_image, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth, const std::chrono::time_point<std::chrono::system_clock> timestamp, unsigned int ts_milliseconds);
    bool ProcessInfrared(cv::Mat &infrared_img, const UINT16* pBuffer, int nWidth, int nHeight, const std::chrono::time_point<std::chrono::system_clock> timestamp, unsigned int ts_milliseconds);
    bool ProcessColor(cv::Mat &color_img, IColorFrame *pColorFrame, RGBQUAD *pBuffer, int nWidth, int nHeight, ColorImageFormat imageFormat, const std::chrono::time_point<std::chrono::system_clock> timestamp, unsigned int ts_milliseconds);

    tf2::Vector3 BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);
    bool AcquireBodyFrame(IBodyFrame*& pBodyFrame);

    std::string GetErrorMessage(HRESULT hr);

    // If inImage exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inImage's
    // data with the cv::Mat directly
    // NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
    // NOTE: This does not cover all cases - it should be easy to add new ones as required.
    cv::Mat QImageToCvMat(const QImage& inImage, bool inCloneImageData = true);
    // If inPixmap exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inPixmap's data
    // with the cv::Mat directly
    // NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
    cv::Mat QPixmapToCvMat(const QPixmap& inPixmap, bool inCloneImageData = true);

    QImage cvMatToQImage(const cv::Mat& inMat);
    QPixmap cvMatToQPixmap(const cv::Mat& inMat);

    void updatePointCloud(UINT16* depthBuffer, unsigned char *colorBuffer);

    void updateDepthImageInColorResolution(cv::Mat& depth_image_hd, const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight);

    std::string serializeTimePoint(const std::chrono::time_point<std::chrono::system_clock>& time, const std::string& format, unsigned int ts_milliseconds);

    void setColorParams(const double color_cx, const double color_cy, const double color_fx, const double color_fy);
    void setIRParams(const double ir_cx, const double ir_cy, const double ir_fx, const double ir_fy, const double ir_k1, const double ir_k2, const double ir_k3, const double ir_p1, const double ir_p2);

    bool m_saveFramesToDisk;

    cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr, cameraMatrixDepth, distortionDepth;
    cv::Mat rotation, translation;

    float m_focalLengthX;
    float m_focalLengthY;
    float m_principalPointX;
    float m_principalPointY;
    float m_radialDistortionSecondOrder;
    float m_radialDistortionFourthOrder;
    float m_radialDistortionSixthOrder;

    int m_colorWidth, m_colorHeight;

    static const int cDepthWidth = 512;
    static const int cDepthHeight = 424;

    static const int cInfraredWidth = 512;
    static const int cInfraredHeight = 424;

    static const int cColorWidth = 1920;
    static const int cColorHeight = 1080;

    static const int cColorBytesPerPixel = 4;

    static constexpr unsigned int cMaxTrackedUsers = 6;

    // InfraredSourceValueMaximum is the highest value that can be returned in the InfraredFrame.
    // It is cast to a float for readability in the visualization code.
    static constexpr float InfraredSourceValueMaximum = static_cast<float>(USHRT_MAX);

    // The InfraredOutputValueMinimum value is used to set the lower limit, post processing, of the
    // infrared data that we will render.
    // Increasing or decreasing this value sets a brightness "wall" either closer or further away.
    static constexpr float InfraredOutputValueMinimum = 0.01f;

    // The InfraredOutputValueMaximum value is the upper limit, post processing, of the
    // infrared data that we will render.
    static constexpr float InfraredOutputValueMaximum = 1.0f;

    // The InfraredSceneValueAverage value specifies the average infrared value of the scene.
    // This value was selected by analyzing the average pixel intensity for a given scene.
    // Depending on the visualization requirements for a given application, this value can be
    // hard coded, as was done here, or calculated by averaging the intensity for each pixel prior
    // to rendering.
    static constexpr float InfraredSceneValueAverage = 0.08f;

    /// The InfraredSceneStandardDeviations value specifies the number of standard deviations
    /// to apply to InfraredSceneValueAverage. This value was selected by analyzing data
    /// from a given scene.
    /// Depending on the visualization requirements for a given application, this value can be
    /// hard coded, as was done here, or calculated at runtime.
    static constexpr float InfraredSceneStandardDeviations = 3.0f;

    bool m_intrinsicsRead;

    // Current Kinect
    IKinectSensor* m_pKinectSensor;
    IMultiSourceFrameReader* m_pMultiSourceFrameReader;
    ICoordinateMapper* m_pCoordinateMapper;

    ColorSpacePoint* m_depth2RGB;
    CameraSpacePoint* m_depth2XYZ;
    DepthSpacePoint* m_pDepthCoordinates;

    bool m_useMultiSourceFrameReader;

    bool m_readImages;
    std::map<std::string, cv::Mat> m_colorImages;
    std::map<int, std::string> m_colorImagesOrdering;
    std::map<std::string, cv::Mat> m_depthImages;
    std::map<int, std::string> m_depthImagesOrdering;
    std::map<std::string, cv::Mat> m_irImages;
    std::map<int, std::string> m_irImagesOrdering;

    int m_currentImageSetIndex;

    // Body reader
    IBodyFrameReader* m_pBodyFrameReader;

    // Depth reader
    IDepthFrameReader* m_pDepthFrameReader;
    RGBQUAD* m_pDepthRGBX;

    // Color reader
    IColorFrameReader* m_pColorFrameReader;
    RGBQUAD* m_pColorRGBX;

    // Infrared reader
    IInfraredFrameReader* m_pInfraredFrameReader;
    RGBQUAD* m_pInfraredRGBX;

    cv::Mat last_depth_img;
    cv::Mat last_depth_img_hd;
    cv::Mat last_infrared_img;
    cv::Mat last_color_img;
    // cv::Mat last_color_img_sd;

    unsigned char* rgb_image;

    std::map<unsigned int, Kinect2SkeletonData> m_trackedUsers;
    std::vector<bool> m_userTrackingStatus;

    sensor_msgs::PointCloud2::Ptr m_pointCloudMsg;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_pointCloud;
    unsigned long m_pointCloudSeqNum;

    ros::Publisher m_pointCloudPub;

    ros::Publisher m_colorImagePub;
    ros::Publisher m_depthImagePub;
    ros::Publisher m_irImagePub;
    ros::Publisher m_registeredDepthImagePub;

    ros::Publisher m_colorImageInDepthResolutionPub;
    
    ros::Publisher m_bodyFramesPub;
    ros::Publisher m_trackingStatesPub;
    ros::NodeHandle m_rosNode;

    bool color_img_received;
    bool infrared_img_received;
    bool depth_img_received;
    bool body_frames_received;

    //libfreenect2::Registration* registration;
    Freenect::FreenectRegistration* registration;
    Freenect::Freenect2Device::ColorCameraParams colorParams;
    Freenect::Freenect2Device::IrCameraParams irParams;

    bool m_irParamsSet;
    bool m_colorParamsSet;

#if 0
    libfreenect2::PacketPipeline* packetPipeline;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device* device;
#endif
};
