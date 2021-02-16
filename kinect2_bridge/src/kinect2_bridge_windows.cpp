#include "kinect2_bridge/kinect2_bridge_private_windows.h"
#include "kinect2_registration/kinect2_console.h"

#include <string>

Kinect2BridgePrivate::Kinect2BridgePrivate():
    m_intrinsicsRead(false),
    m_useMultiSourceFrameReader(false),
    m_pKinectSensor(nullptr),
    m_pMultiSourceFrameReader(nullptr),
    m_pCoordinateMapper(nullptr),
    m_focalLengthX(0.0f),
    m_focalLengthY(0.0f),
    m_principalPointX(0.0f),
    m_principalPointY(0.0f),
    m_radialDistortionSecondOrder(0.0f),
    m_radialDistortionFourthOrder(0.0f),
    m_radialDistortionSixthOrder(0.0f),
    m_colorWidth(0),
    m_colorHeight(0),
    m_pInfraredRGBX(nullptr),
    m_pDepthRGBX(nullptr),
    m_pColorRGBX(nullptr),
    m_depth2RGB(nullptr),
    m_depth2XYZ(nullptr),
    rgb_image(nullptr),
    m_pointCloudSeqNum(0),
    color_img_received(false),
    infrared_img_received(false),
    depth_img_received(false),
    body_frames_received(false)
{

}

std::string Kinect2BridgePrivate::GetErrorMessage(HRESULT hr)
{
    LPTSTR errorText = NULL;
    std::string retVal;
    FormatMessage(
                // use system message tables to retrieve error text
                FORMAT_MESSAGE_FROM_SYSTEM
                // allocate buffer on local heap for error text
                |FORMAT_MESSAGE_ALLOCATE_BUFFER
                // Important! will fail otherwise, since we're not
                // (and CANNOT) pass insertion parameters
                |FORMAT_MESSAGE_IGNORE_INSERTS,
                NULL,    // unused with FORMAT_MESSAGE_FROM_SYSTEM
                hr,
                MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                (LPTSTR)&errorText,  // output
                0, // minimum size for output buffer
                NULL);   // arguments - see note

    if ( NULL != errorText )
    {
        retVal = std::string(errorText);
        // release memory allocated by FormatMessage()
        LocalFree(errorText);
        errorText = NULL;
    }

    return retVal;
}

bool Kinect2BridgePrivate::initDevice(const std::string& sensor)
{
    HRESULT hr;

    // Handle default sensor only for now
    if (!sensor.empty())
        std::cerr << "Win32 supports default Kinect2 only for now." << std::endl;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);

    if (FAILED(hr))
    {
        ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Could not open default Kinect2 sensor: " << GetErrorMessage(hr));
        return false;
    }

    if (m_pKinectSensor)
    {
        HRESULT hr_kinect_opened = m_pKinectSensor->Open();

        if (SUCCEEDED(hr_kinect_opened))
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Successfully opened Kinect2 device.");

            HRESULT hr_coord_mapper = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
            if (SUCCEEDED(hr_coord_mapper))
            {
                m_depth2RGB = new ColorSpacePoint[cDepthWidth * cDepthHeight];
                m_depth2XYZ = new CameraSpacePoint[cDepthWidth * cDepthHeight];
                m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];

                rgb_image = new unsigned char[cColorWidth * cColorHeight * 4];

                m_pointCloudPub = m_rosNode.advertise<sensor_msgs::PointCloud2>("/kinect2/point_cloud", 10);

                m_userTrackingStatus.resize(cMaxTrackedUsers);
                for (size_t k = 0; k < cMaxTrackedUsers; k++)
                    m_userTrackingStatus[k] = false;

                m_bodyFramesPub = m_rosNode.advertise<bb_kinect2_msgs::Person>("/kinect2/tracking_data", 10);
                m_trackingStatesPub = m_rosNode.advertise<bb_kinect2_msgs::TrackingStates>("/kinect2/tracking_states", 10);

                m_pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
                m_pointCloud->width = static_cast<uint32_t>(cDepthWidth);
                m_pointCloud->height = static_cast<uint32_t>(cDepthHeight);
                m_pointCloud->points.resize(m_pointCloud->height * m_pointCloud->width);
                m_pointCloud->is_dense = false;
            }

            if (m_useMultiSourceFrameReader)
            {
                HRESULT hr_reader = m_pKinectSensor->OpenMultiSourceFrameReader(
                            FrameSourceTypes_Depth |
                            FrameSourceTypes_Infrared |
                            FrameSourceTypes_Color |
                            FrameSourceTypes_Body,
                            &m_pMultiSourceFrameReader
                            );

                if (FAILED(hr_reader))
                {
                    ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Failed to create MultiSourceFrameReader instance: " << GetErrorMessage(hr_reader));
                    return false;
                }
            }
            else
            {
                // Initialize the Kinect and get coordinate mapper and the body reader
                IBodyFrameSource* pBodyFrameSource = NULL;
                HRESULT hr;
                if (SUCCEEDED(hr_kinect_opened))
                {
                    hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
                }

                SafeRelease(pBodyFrameSource);

                // Get the color reader
                IColorFrameSource* pColorFrameSource = NULL;

                if (SUCCEEDED(hr_kinect_opened))
                {
                    hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);

                    IFrameDescription* frameDesc = NULL;
                    pColorFrameSource->get_FrameDescription(&frameDesc);

                    frameDesc->get_Width(&m_colorWidth);
                    frameDesc->get_Height(&m_colorHeight);

                    SafeRelease(frameDesc);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);

                    // create heap storage for color pixel data in RGBX format
                    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
                }

                SafeRelease(pColorFrameSource);

                // Get the depth reader
                IDepthFrameSource* pDepthFrameSource = NULL;

                if (SUCCEEDED(hr_kinect_opened))
                {
                    hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
                    // create heap storage for depth pixel data in RGBX format
                    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
                }

                SafeRelease(pDepthFrameSource);

                // Get an infrared frame reader instance
                IInfraredFrameSource* pInfraredFrameSource = NULL;

                if (SUCCEEDED(hr))
                {
                    hr = m_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);

                    IFrameDescription* frameDesc = NULL;
                    pInfraredFrameSource->get_FrameDescription(&frameDesc);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pInfraredFrameSource->OpenReader(&m_pInfraredFrameReader);

                    // create heap storage for infrared pixel data in RGBX format
                    m_pInfraredRGBX = new RGBQUAD[cInfraredWidth * cInfraredHeight];
                }

                SafeRelease(pInfraredFrameSource);
            }

            CameraIntrinsics cameraIntrinsics;
            m_pCoordinateMapper->GetDepthCameraIntrinsics(&cameraIntrinsics);
            m_focalLengthX = cameraIntrinsics.FocalLengthX;
            m_focalLengthY = cameraIntrinsics.FocalLengthY;
            m_principalPointX = cameraIntrinsics.PrincipalPointX;
            m_principalPointY = cameraIntrinsics.PrincipalPointY;
            m_radialDistortionSecondOrder = cameraIntrinsics.RadialDistortionSecondOrder;
            m_radialDistortionFourthOrder = cameraIntrinsics.RadialDistortionFourthOrder;
            m_radialDistortionSixthOrder = cameraIntrinsics.RadialDistortionSixthOrder;

            m_intrinsicsRead = true;

            return true;
        }
    }

    std::cerr << "Failed to open default Kinect2 device!" << std::endl;
    return false;
}

bool Kinect2BridgePrivate::shutdownDevice()
{
    if (m_pKinectSensor)
    {
        BOOLEAN kinectIsOpen;
        HRESULT hr = m_pKinectSensor->get_IsOpen(&kinectIsOpen);
        if (SUCCEEDED(hr))
        {
            SafeRelease(m_pCoordinateMapper);
            if (m_useMultiSourceFrameReader)
                SafeRelease(m_pMultiSourceFrameReader);

            hr = m_pKinectSensor->Close();
            if (SUCCEEDED(hr))
            {
                return true;
            }
        }

        if (!m_useMultiSourceFrameReader)
        {
            if (m_pBodyFrameReader)
            {
                SafeRelease(m_pBodyFrameReader);
            }

            if (m_pColorFrameReader)
            {
                SafeRelease(m_pColorFrameReader);
            }

            if (m_pDepthFrameReader)
            {
                SafeRelease(m_pDepthFrameReader);
            }

            if (m_pInfraredFrameReader)
            {
                SafeRelease(m_pInfraredFrameReader);
            }
        }

        if (m_pDepthRGBX)
        {
            delete[] m_pDepthRGBX;
            m_pDepthRGBX = nullptr;
        }

        if (m_pInfraredRGBX)
        {
            delete[] m_pInfraredRGBX;
            m_pInfraredRGBX = nullptr;
        }

        if (m_pColorRGBX)
        {
            delete[] m_pColorRGBX;
            m_pColorRGBX = nullptr;
        }

        if (m_depth2RGB)
        {
            delete m_depth2RGB;
            m_depth2RGB = nullptr;
        }

        if (m_depth2XYZ)
        {
            delete m_depth2XYZ;
            m_depth2XYZ = nullptr;
        }

        if (m_pDepthCoordinates)
        {
            delete[] m_pDepthCoordinates;
            m_pDepthCoordinates = nullptr;
        }

        if (rgb_image)
        {
            delete[] rgb_image;
            rgb_image = nullptr;
        }
    }

    return false;
}

// Same here: Only default sensor available, so acknowledge start/stop device by validity of Kinect2 instance pointer
// Start/shutdown is done in initDevice()/shutdownDevice()
bool Kinect2BridgePrivate::startDevice(const std::string& sensor)
{
    if (m_pKinectSensor)
        return true;

    // TODO: Resume private Win32 device reading loop

    return false;
}

bool Kinect2BridgePrivate::stopDevice(const std::string& sensor)
{
    if (m_pKinectSensor)
        return true;

    // TODO: Pause private Win32 device reading loop

    return false;
}

// Only GetDefaultKinectSensor() available on Win32!
int Kinect2BridgePrivate::getNumDevices()
{
    if (!m_pKinectSensor)
        return 0;

    return 1;
}

// Same here: Only default sensor available, so acknowledge device presence if it's open regardless of dev. identifier
bool Kinect2BridgePrivate::locateDevice(const std::string& sensor)
{
    if (m_pKinectSensor)
        return true;

    return false;
}

// Same here: Only default sensor available, so acknowledge if it's open regardless of dev. identifier
bool Kinect2BridgePrivate::openDevice(const std::string& sensor)
{
    if (m_pKinectSensor)
        return true;

    return false;
}

// Same here: Only default sensor available, so acknowledge if it's closed regardless of dev. identifier
bool Kinect2BridgePrivate::closeDevice(const std::string& sensor, bool closeOnly)
{
    if (!m_pKinectSensor)
        return true;

    return false;
}

// Using the Kinect2 SDK depth/color alignment method under Win32; this is redundant then...
void Kinect2BridgePrivate::createCameraParameters(const std::string& sensor)
{
    cameraMatrixColor.at<double>(0, 0) = m_focalLengthX;
    cameraMatrixColor.at<double>(1, 1) = m_focalLengthY;
    cameraMatrixColor.at<double>(0, 2) = m_principalPointX;
    cameraMatrixColor.at<double>(1, 2) = m_principalPointY;
    cameraMatrixColor.at<double>(2, 2) = 1;

    cameraMatrixIr.at<double>(0, 0) = m_focalLengthX;
    cameraMatrixIr.at<double>(1, 1) = m_focalLengthY;
    cameraMatrixIr.at<double>(0, 2) = m_principalPointX;
    cameraMatrixIr.at<double>(1, 2) = m_principalPointY;
    cameraMatrixIr.at<double>(2, 2) = 1;

    distortionIr.at<double>(0, 0) = m_radialDistortionSecondOrder;
    distortionIr.at<double>(0, 1) = m_radialDistortionFourthOrder;
    distortionIr.at<double>(0, 2) = 0.0; //???
    distortionIr.at<double>(0, 3) = 0.0; //???
    distortionIr.at<double>(0, 4) = m_radialDistortionSixthOrder;
}

bool Kinect2BridgePrivate::initRegistration(const std::string &method, const int32_t device, const double maxDepth)
{
    return true;
}

bool Kinect2BridgePrivate::initPipeline(const std::string &method, const int32_t device)
{
    return true;
}

void Kinect2BridgePrivate::initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth)
{

}

// No separate steps necessary with the Win32 Kinect2 SDK; once the MultiSourceFrameReader is instantiated we can query it whenever required
bool Kinect2BridgePrivate::startStreams(bool isSubscribedColor, bool isSubscribedDepth)
{
    return true;
}

bool Kinect2BridgePrivate::receiveIrDepth(IMultiSourceFrame*& frame, IInfraredFrame*& pInfraredFrame, IDepthFrame*& pDepthFrame)
{
    bool irReceived = false, depthReceived = false;
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "In receiveIrDepth()");
    if (m_useMultiSourceFrameReader)
    {
        if (frame == nullptr)
        {
            ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Invalid MultiSourceFrameReader reference received!");
            return false;
        }

        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing depth & IR frames from MultiSourceFrameReader.");
        IInfraredFrameReference* pInfraredFrameReference = NULL;
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Retrieving IR frame reference...");
        HRESULT hr = frame->get_InfraredFrameReference(&pInfraredFrameReference);
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "IR frame reference received.");

        if (SUCCEEDED(hr))
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing IR frame.");
            IInfraredFrame* pInfraredFrame = NULL;
            INT64 nTime = 0;
            IFrameDescription* pFrameDescription = NULL;
            int nWidth = 0;
            int nHeight = 0;
            UINT nBufferSize = 0;
            UINT16* pBuffer = NULL;

            HRESULT hr = pInfraredFrameReference->AcquireFrame(&pInfraredFrame);

            if (SUCCEEDED(hr))
            {
                hr = pInfraredFrame->get_RelativeTime(&nTime);
            }

            if (SUCCEEDED(hr))
            {
                hr = pInfraredFrame->get_FrameDescription(&pFrameDescription);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Width(&nWidth);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Height(&nHeight);
            }

            if (SUCCEEDED(hr))
            {
                hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            }

            if (SUCCEEDED(hr))
            {
                if (ProcessInfrared(last_infrared_img, pBuffer, nWidth, nHeight))
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "IR frame processed.");
                    irReceived = true;
                }
                else
                {
                    ROS_WARN_STREAM_NAMED("kinect2_bridge", "Failed to process IR frame!");
                    irReceived = false;
                }
            }
        }

        IDepthFrameReference* pDepthFrameReference = NULL;

        hr = frame->get_DepthFrameReference(&pDepthFrameReference);

        if (SUCCEEDED(hr))
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing depth frame.");
            INT64 nTime = 0;
            IDepthFrame* pDepthFrame = NULL;
            IFrameDescription* pFrameDescription = NULL;
            int nWidth = 0;
            int nHeight = 0;
            USHORT nDepthMinReliableDistance = 0;
            USHORT nDepthMaxDistance = 0;
            UINT nBufferSize = 0;
            UINT16* pBuffer = NULL;

            hr = pDepthFrameReference->get_RelativeTime(&nTime);

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
            }

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Width(&nWidth);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Height(&nHeight);
            }

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
            }

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
            }

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            }

            if (SUCCEEDED(hr))
            {
                if (ProcessDepth(last_depth_img, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance))
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Depth frame processed.");
                    depthReceived = true;
                }
                else
                {
                    ROS_WARN_STREAM_NAMED("kinect2_bridge", "Depth frame processing failed!");
                    depthReceived = false;
                }
            }

            SafeRelease(pFrameDescription);
            SafeRelease(pDepthFrame);
        }

        if (pInfraredFrameReference)
            pInfraredFrameReference->Release();

        if (pDepthFrameReference)
            pDepthFrameReference->Release();

        return (irReceived && depthReceived);
    }
    else
    {
        if (pInfraredFrame)
        {
            INT64 nTime = 0;
            IFrameDescription* pFrameDescription = NULL;
            int nWidth = 0;
            int nHeight = 0;
            UINT nBufferSize = 0;
            UINT16* pBuffer = NULL;

            HRESULT hr = pInfraredFrame->get_RelativeTime(&nTime);

            if (SUCCEEDED(hr))
            {
                hr = pInfraredFrame->get_FrameDescription(&pFrameDescription);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Width(&nWidth);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Height(&nHeight);
            }

            if (SUCCEEDED(hr))
            {
                hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            }

            if (SUCCEEDED(hr))
            {
                if (ProcessInfrared(last_infrared_img, pBuffer, nWidth, nHeight))
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "IR frame processed.");
                    irReceived = true;
                }
                else
                {
                    ROS_WARN_STREAM_NAMED("kinect2_bridge", "Failed to process IR frame!");
                    irReceived = false;
                }
            }

            SafeRelease(pFrameDescription);
        }

        if (pDepthFrame)
        {
            INT64 nTime = 0;
            IFrameDescription* pFrameDescription = NULL;
            int nWidth = 0;
            int nHeight = 0;
            USHORT nDepthMinReliableDistance = 0;
            USHORT nDepthMaxDistance = 0;
            UINT nBufferSize = 0;
            UINT16* pBuffer = NULL;

            HRESULT hr = pDepthFrame->get_FrameDescription(&pFrameDescription);

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Width(&nWidth);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Height(&nHeight);
            }

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
            }

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
            }

            if (SUCCEEDED(hr))
            {
                hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            }

            if (SUCCEEDED(hr))
            {
                if (ProcessDepth(last_depth_img, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance))
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Depth frame processed.");
                    depthReceived = true;
                }
                else
                {
                    ROS_WARN_STREAM_NAMED("kinect2_bridge", "Depth frame processing failed!");
                    depthReceived = false;
                }
            }
        }

        return (irReceived && depthReceived);
    }
    return false;
}

bool Kinect2BridgePrivate::receiveColor(IMultiSourceFrame *&frame, IColorFrame *&pColorFrame)
{
    bool colorReceived = false;

    if (m_useMultiSourceFrameReader && frame)
    {
        IColorFrameReference* pColorFrameReference = NULL;

        HRESULT hr = frame->get_ColorFrameReference(&pColorFrameReference);

        if (SUCCEEDED(hr))
        {
            IColorFrame* pColorFrame = NULL;

            hr = pColorFrameReference->AcquireFrame(&pColorFrame);

            if (SUCCEEDED(hr))
            {
                INT64 nTime = 0;
                IFrameDescription* pFrameDescription = NULL;
                int nWidth = 0;
                int nHeight = 0;
                ColorImageFormat imageFormat = ColorImageFormat_None;
                UINT nBufferSize = 0;
                RGBQUAD* pBuffer = NULL;

                hr = pColorFrame->get_RelativeTime(&nTime);

                if (SUCCEEDED(hr))
                {
                    hr = pColorFrame->get_FrameDescription(&pFrameDescription);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pFrameDescription->get_Width(&nWidth);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pFrameDescription->get_Height(&nHeight);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
                }

                if (SUCCEEDED(hr))
                {
                    if (ProcessColor(last_color_img, pColorFrame, m_pColorRGBX, nWidth, nHeight, imageFormat))
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Successfully converted color frame to cv::Mat (multi-frame read).");
                        colorReceived = true;
                    }
                    else
                    {
                        ROS_WARN_STREAM_NAMED("kinect2_bridge", "Conversion from color frame to cv::Mat FAILED (multi-frame read)!");
                        colorReceived = false;
                    }

                    /*if (status[COLOR_SD_RECT])
                    {
                        lockRegSD.lock();
                        memcpy(this->color.data, colorFrame->data, sizeColor.width * sizeColor.height * 4);
                        this->color.format = colorFrame->format;
                        lockRegSD.unlock();
                    }
                    if (status[COLOR_HD] || status[COLOR_HD_RECT] || status[COLOR_QHD] || status[COLOR_QHD_RECT] ||
                       status[MONO_HD] || status[MONO_HD_RECT] || status[MONO_QHD] || status[MONO_QHD_RECT])
                    {
                        cv::Mat tmp;
                        cv::flip(color, tmp, 1);
                        if(colorFrame->format == libfreenect2::Frame::BGRX)
                        {
                            cv::cvtColor(tmp, images[COLOR_HD], CV_BGRA2BGR);
                        }
                        else
                        {
                            cv::cvtColor(tmp, images[COLOR_HD], CV_RGBA2BGR);
                        }
                    }*/
                }

                SafeRelease(pFrameDescription);
            }
        }

        if (pColorFrameReference)
            pColorFrameReference->Release();
    }
    else
    {
        if (pColorFrame)
        {
            INT64 nTime = 0;
            IFrameDescription* pFrameDescription = NULL;
            int nWidth = 0;
            int nHeight = 0;
            ColorImageFormat imageFormat = ColorImageFormat_None;
            UINT nBufferSize = 0;
            RGBQUAD* pBuffer = NULL;

            HRESULT hr = pColorFrame->get_RelativeTime(&nTime);

            if (SUCCEEDED(hr))
            {
                hr = pColorFrame->get_FrameDescription(&pFrameDescription);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Width(&nWidth);
            }

            if (SUCCEEDED(hr))
            {
                hr = pFrameDescription->get_Height(&nHeight);
            }

            if (SUCCEEDED(hr))
            {
                hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
            }

            if (SUCCEEDED(hr))
            {
                if (ProcessColor(last_color_img, pColorFrame, m_pColorRGBX, nWidth, nHeight, imageFormat))
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Successfully converted color frame to cv::Mat (single frame read).");
                    colorReceived = true;
                }
                else
                {
                    ROS_WARN_STREAM_NAMED("kinect2_bridge", "Conversion from color frame to cv::Mat FAILED (single frame read)!");
                    colorReceived = false;
                }
            }

            SafeRelease(pFrameDescription);
        }
    }

    return colorReceived;
}

void Kinect2BridgePrivate::updatePointCloud(UINT16 *depthBuffer, unsigned char* colorBuffer)
{
    // Reset Point Cloud
    m_pointCloud->clear();

    if (!m_pointCloudMsg)
        m_pointCloudMsg.reset(new sensor_msgs::PointCloud2());

    // Convert to Point Cloud
    for (int depthY = 0; depthY < cDepthHeight; depthY++)
    {
        for (int depthX = 0; depthX < cDepthWidth; depthX++)
        {
            pcl::PointXYZRGBA point;

            DepthSpacePoint depthSpacePoint = { static_cast<float>(depthX), static_cast<float>(depthY) };
            UINT16 depth = depthBuffer[depthY * cDepthWidth + depthX];
            ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };

            // Retrieve Mapped Coordinates
            if (SUCCEEDED(m_pCoordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint)) &&
                SUCCEEDED(m_pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint)))
            {
                // Set Color to Point
                int colorX = static_cast<int>(colorSpacePoint.X + 0.5f);
                int colorY = static_cast<int>(colorSpacePoint.Y + 0.5f);
                if ((0 <= colorX) && (colorX < cColorWidth) && (0 <= colorY) && (colorY < cColorHeight))
                {
                    unsigned int colorIndex = (colorY * cColorWidth + colorX) * cColorBytesPerPixel;
                    point.b = colorBuffer[colorIndex + 0];
                    point.g = colorBuffer[colorIndex + 1];
                    point.r = colorBuffer[colorIndex + 2];
                    point.a = colorBuffer[colorIndex + 3];
                }

                // Set Depth to Point
                if((0 <= colorX) && (colorX < cColorWidth) && (0 <= colorY) && (colorY < cColorHeight))
                {
                    point.x = cameraSpacePoint.X;
                    point.y = cameraSpacePoint.Y;
                    point.z = cameraSpacePoint.Z;
                }

                // Set Point to Point Cloud
                m_pointCloud->push_back(point);
            }
        }
    }

    pcl::toROSMsg(*m_pointCloud.get(), *m_pointCloudMsg.get());

    m_pointCloudMsg->header.stamp = ros::Time::now();
    m_pointCloudMsg->header.seq = m_pointCloudSeqNum;
    m_pointCloudMsg->header.frame_id = "kinect2_ir_optical_frame";

    m_pointCloudSeqNum++;

    m_pointCloudPub.publish(m_pointCloudMsg);
}


bool Kinect2BridgePrivate::receiveFrames()
{
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "m_d->receiveFrames() (WIN32)");
    bool newFrames = false;
    IMultiSourceFrame* frame = NULL;
    IInfraredFrame* infraredFrame = NULL;
    IDepthFrame* depthFrame = NULL;
    IColorFrame* colorFrame = NULL;
    IBodyFrame* bodyFrame = NULL;

    HRESULT hr_multi_frame_read = E_FAIL;
    HRESULT hr_body_frame_read = E_FAIL;
    HRESULT hr_depth_frame_read = E_FAIL;
    HRESULT hr_ir_frame_read = E_FAIL;
    HRESULT hr_color_frame_read = E_FAIL;

    bool processMultiFrame = true;
    // Read all at once
    if (m_useMultiSourceFrameReader)
    {
        hr_multi_frame_read = m_pMultiSourceFrameReader->AcquireLatestFrame(&frame);
        if (FAILED(hr_multi_frame_read))
        {
            if (hr_multi_frame_read == E_PENDING)
            {
                ROS_INFO_STREAM_NAMED("kinect2_bridge", "Not processing multi-frame data, Kinect2 read timed out. frame = " << frame);
            }
            else
            {
                ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Could not read multi-source frame set from Kinect 2: " << GetErrorMessage(hr_multi_frame_read) << " (" << hr_multi_frame_read << ")");
            }
            processMultiFrame = false;
            newFrames = false;
        }
        else
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "MultiSourceFrameReader frame acquired, frame = " << frame);
            // newFrames = true;
        }
    }
    // Read individual frames
    else
    {
        hr_body_frame_read = m_pBodyFrameReader->AcquireLatestFrame(&bodyFrame);

        if (FAILED(hr_body_frame_read))
        {
            ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Could not read body frame from Kinect 2: " << GetErrorMessage(hr_body_frame_read) << " (" << hr_body_frame_read << ")");
        }

        hr_depth_frame_read = m_pDepthFrameReader->AcquireLatestFrame(&depthFrame);
        if (FAILED(hr_depth_frame_read))
        {
            ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Could not read depth frame from Kinect 2: " << GetErrorMessage(hr_depth_frame_read) << " (" << hr_depth_frame_read << ")");
        }

        hr_ir_frame_read = m_pInfraredFrameReader->AcquireLatestFrame(&infraredFrame);
        if (FAILED(hr_ir_frame_read))
        {
            ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Could not read infrared frame from Kinect 2: " << GetErrorMessage(hr_ir_frame_read) << " (" << hr_ir_frame_read << ")");
        }

        hr_color_frame_read = m_pColorFrameReader->AcquireLatestFrame(&colorFrame);
        if (FAILED(hr_color_frame_read))
        {
            ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Could not read color frame from Kinect 2: " << GetErrorMessage(hr_color_frame_read) << " (" << hr_color_frame_read << ")");
        }
    }

#if 1
    bool bodyReceived = false;
    bool infraredReceived = false;
    bool colorReceived = false;
    if (!m_useMultiSourceFrameReader)
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Successfully read next set of frames from Kinect 2 (using individual readers).");

        if (SUCCEEDED(hr_body_frame_read))
        {
            bodyReceived = AcquireBodyFrame(bodyFrame);
            body_frames_received = bodyReceived;
            SafeRelease(bodyFrame);
        }

        if (SUCCEEDED(hr_ir_frame_read) && SUCCEEDED(hr_depth_frame_read))
        {
            infraredReceived = receiveIrDepth(frame, infraredFrame, depthFrame);
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "IR & depth processed: " << (int) infraredReceived);
            if (SUCCEEDED(hr_ir_frame_read))
                SafeRelease(infraredFrame);

        }

        if (SUCCEEDED(hr_color_frame_read))
        {
            colorReceived = receiveColor(frame, colorFrame);
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Color processed: " << (int) colorReceived);
        }

        if (SUCCEEDED(hr_depth_frame_read) && SUCCEEDED(hr_color_frame_read))
        {
            if (SUCCEEDED(colorFrame->CopyConvertedFrameDataToArray(cColorWidth * cColorHeight * 4, rgb_image, ColorImageFormat_Rgba)))
            {
                unsigned int depthBufSize;
                UINT16* depthBuf;
                if (SUCCEEDED(depthFrame->AccessUnderlyingBuffer(&depthBufSize, &depthBuf)))
                {
                    updatePointCloud(depthBuf, rgb_image);
                }
            }
        }

        if (SUCCEEDED(hr_depth_frame_read))
            SafeRelease(depthFrame);

        if (SUCCEEDED(hr_color_frame_read))
            SafeRelease(colorFrame);

        color_img_received = colorReceived;
        infrared_img_received = infraredReceived;
        depth_img_received = infraredReceived;

        if (infraredReceived || colorReceived || bodyReceived)
            newFrames = true;
    }
    else
    {
        if (!processMultiFrame)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Skipping multi-frame processing, Kinect2 read cycle timed out!");
            newFrames = false;
        }
        else
        {
            // ROS_INFO_STREAM_NAMED("kinect2_bridge", "Successfully read next set of frames from Kinect 2 (using MultiSourceFrameReader).");

            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing color frame.");
            colorReceived = receiveColor(frame, colorFrame);
#if 0
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing IR & depth frames.");
            infraredReceived = receiveIrDepth(frame, infraredFrame, depthFrame);
            if (frame != nullptr)
            {
                IBodyFrameReference* bodyFrameRef = nullptr;
                HRESULT hr_body_received = frame->get_BodyFrameReference(&bodyFrameRef);
                if (SUCCEEDED(hr_body_received))
                {
                    HRESULT hr_body_frame_received = bodyFrameRef->AcquireFrame(&bodyFrame);
                    if (SUCCEEDED(hr_body_frame_received))
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing body frame.");
                        bodyReceived = AcquireBodyFrame(bodyFrame);

                        SafeRelease(bodyFrame);
                    }
                    SafeRelease(bodyFrameRef);
                }
            }
#endif
            frame->Release();
            body_frames_received = bodyReceived;
            newFrames = colorReceived || infraredReceived || bodyReceived;
        }
    }
#endif
    return newFrames;
}

void Kinect2BridgePrivate::processColorDepthFrame()
{

}

bool Kinect2BridgePrivate::ProcessDepth(cv::Mat& depth_map_image, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
    // Make sure we've received valid data
    if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
    {
        RGBQUAD* pRGBX = m_pDepthRGBX;

        static unsigned int depthImgCounter = 0;

        int lineNum = 0;
        int rowNum = 0;

        depth_map_image = cv::Mat(nHeight, nWidth, CV_16UC1);
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "cv::Mat size for depth map: " << depth_map_image.rows << "x" << depth_map_image.cols);

        // end pixel is start + width*height - 1
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

        while (pBuffer < pBufferEnd)
        {
            USHORT depth = *pBuffer;

            // To convert to a byte, we're discarding the most-significant
            // rather than least-significant bits.
            // We're preserving detail, although the intensity will "wrap."
            // Values outside the reliable depth range are mapped to 0 (black).

            // Note: Using conditionals in this loop could degrade performance.
            // Consider using a lookup table instead when writing production code.
            BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

            pRGBX->rgbRed = intensity;
            pRGBX->rgbGreen = intensity;
            pRGBX->rgbBlue = intensity;

            depth_map_image.at<uint16_t>(lineNum, rowNum) = intensity;

            rowNum++;
            if (rowNum % nWidth == 0)
                lineNum++;

            if (rowNum >= nWidth)
                rowNum = 0;

            ++pRGBX;
            ++pBuffer;
        }

        depthImgCounter++;

        if (depthImgCounter % 32 == 0)
        {
            // cv::imwrite("T:\\temp\\depth_img.jpg", depth_map_image);
            depthImgCounter = 0;
        }

        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Received new depth data of size " << nWidth << "x" << nHeight);
        return true;
    }
    return false;
}

bool Kinect2BridgePrivate::ProcessInfrared(cv::Mat& infrared_img, const UINT16* pBuffer, int nWidth, int nHeight)
{
    if (m_pInfraredRGBX && pBuffer && (nWidth == cInfraredWidth) && (nHeight == cInfraredHeight))
    {
        RGBQUAD* pDest = m_pInfraredRGBX;

        static unsigned int infraredImgCounter = 0;

        unsigned int lineNum = 0;
        unsigned int rowNum = 0;

        infrared_img = cv::Mat(nHeight, nWidth, CV_16UC1);
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "cv::Mat size for infrared image: " << infrared_img.rows << "x" << infrared_img.cols);

        // end pixel is start + width*height - 1
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

        while (pBuffer < pBufferEnd)
        {
            // normalize the incoming infrared data (ushort) to a float ranging from
            // [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
            // 1. dividing the incoming value by the source maximum value
            float intensityRatio = static_cast<float>(*pBuffer) / InfraredSourceValueMaximum;

            // 2. dividing by the (average scene value * standard deviations)
            intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;

            // 3. limiting the value to InfraredOutputValueMaximum
            intensityRatio = std::min(InfraredOutputValueMaximum, intensityRatio);

            // 4. limiting the lower value InfraredOutputValueMinimym
            intensityRatio = std::max(InfraredOutputValueMinimum, intensityRatio);

            // 5. converting the normalized value to a byte and using the result
            // as the RGB components required by the image
            byte intensity = static_cast<byte>(intensityRatio * 255.0f);
            pDest->rgbRed = intensity;
            pDest->rgbGreen = intensity;
            pDest->rgbBlue = intensity;

            infrared_img.at<uint16_t>(lineNum, rowNum) = intensity;

            rowNum++;
            if (rowNum % nWidth == 0)
                lineNum++;

            if (rowNum >= nWidth)
                rowNum = 0;

            ++pDest;
            ++pBuffer;
        }

        infraredImgCounter++;

        if (infraredImgCounter % 32 == 0)
        {
            // cv::imwrite("T:\\temp\\infrared_img.jpg", infrared_img);
            infraredImgCounter = 0;
        }
        return true;
    }

    return false;
}

tf2::Vector3 Kinect2BridgePrivate::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
    // Calculate the body's position on the screen
    DepthSpacePoint depthPoint = { 0 };
    m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

    float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
    float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

    return tf2::Vector3(screenPointX, screenPointY, 0.0);
}

bool Kinect2BridgePrivate::AcquireBodyFrame(IBodyFrame *&pBodyFrame)
{
    if (!pBodyFrame)
    {
        ROS_WARN_STREAM_NAMED("kinect2_bridge", "No valid BodyFrame received!");
        return false;
    }

    m_trackedUsers.clear();

    INT64 nTime = 0;

    HRESULT hr = pBodyFrame->get_RelativeTime(&nTime);
    IBody* ppBodies[BODY_COUNT] = { 0 };

    if (SUCCEEDED(hr))
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Refreshed BodyFrameReader data in Kinect2Device.");
        hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
    }

    if (SUCCEEDED(hr))
    {
        int width = 640; // TODO: Pass in actual size of screen?
        int height = 480;
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing tracking data for " << _countof(ppBodies) << " tracked humans.");
        int trackedBodies = 0;
        for (unsigned long long i = 0; i < _countof(ppBodies); ++i)
        {
            IBody* pBody = ppBodies[i];
            if (pBody)
            {
                // qDebug() << "Tracked human " << i << " is a VALID entry.";
                BOOLEAN bTracked = false;
                hr = pBody->get_IsTracked(&bTracked);

                ROS_INFO_STREAM_NAMED("kinect2_bridge", "Tracked human " << i << " bTracked = " << (int) bTracked << ", hr = " << (unsigned long) hr);

                if (SUCCEEDED(hr) && bTracked)
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Tracked human " << i << " is actively tracked.");
                    Joint joints[JointType_Count];
                    HandState leftHandState = HandState_Unknown;
                    HandState rightHandState = HandState_Unknown;

                    pBody->get_HandLeftState(&leftHandState);
                    pBody->get_HandRightState(&rightHandState);

                    hr = pBody->GetJoints(_countof(joints), joints);
                    if (SUCCEEDED(hr))
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Tracked human " << i << " has a valid list of joints.");
                        UINT64 trackingID;
                        pBody->get_TrackingId(&trackingID);

                        Kinect2SkeletonData skeletonData;
                        skeletonData.trackingId = i;
                        skeletonData.trackingIndex = trackingID;

                        for (int j = 0; j < _countof(joints); ++j)
                        {
                            skeletonData.joints[j].JointType = joints[j].JointType;
                            skeletonData.joints[j].TrackingState = joints[j].TrackingState;
                            skeletonData.joints[j].Position.X = joints[j].Position.X;
                            skeletonData.joints[j].Position.Y = joints[j].Position.Y;
                            skeletonData.joints[j].Position.Z = joints[j].Position.Z;
                            skeletonData.jointPositions3D[j] = tf2::Vector3(joints[j].Position.X, joints[j].Position.Y, joints[j].Position.Z);
                            skeletonData.jointPositions2D[j] = BodyToScreen(joints[j].Position, width, height);
                        }
                        trackedBodies++;

                        m_trackedUsers[i] = skeletonData;
                        m_userTrackingStatus[i] = true;
                    }
                }
                else
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "No tracking/tracking lost for index " << i << ".");
                    m_userTrackingStatus[i] = false;
                }
            }
        }
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Number of tracked users: " << trackedBodies);
    }

    for (unsigned long long i = 0; i < _countof(ppBodies); ++i)
    {
        SafeRelease(ppBodies[i]);
    }

    return true;
}

bool Kinect2BridgePrivate::ProcessColor(cv::Mat& color_img, IColorFrame* pColorFrame, RGBQUAD* pBuffer, int nWidth, int nHeight, ColorImageFormat imageFormat)
{
    HRESULT hr = E_FAIL;
    UINT nBufferSize = 0;
    if (imageFormat == ColorImageFormat_Bgra)
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "ColorImageFormat_Bgra, using AccessRawUnderlyingBuffer()");
        hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
    }
    else if (m_pColorRGBX)
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "ColorImageFormat_Rgba, using CopyConvertedFrameDataToArray()");
        pBuffer = m_pColorRGBX;
        nBufferSize = nWidth * nHeight * sizeof(RGBQUAD);
        hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
    }

    static unsigned int colorImgCounter = 0;
    if (SUCCEEDED(hr))
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Successfully read a new color frame.");
        // QPixmap colorFrame = QPixmap::fromImage(QImage((uchar*)pBuffer, nWidth, nHeight, QImage::Format_RGB32));

        QImage colorFrame = QImage((uchar*)pBuffer, nWidth, nHeight, QImage::Format_RGB32);
        color_img = QImageToCvMat(colorFrame);

        colorImgCounter++;

        if (colorImgCounter % 32 == 0)
        {
            // cv::imwrite("T:\\temp\\color_img.jpg", color_img);
            colorImgCounter = 0;
        }

        return true;
    }
    else
    {
        return false;
    }
}

// If inPixmap exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inPixmap's data
// with the cv::Mat directly
//    NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
cv::Mat Kinect2BridgePrivate::QPixmapToCvMat(const QPixmap& inPixmap, bool inCloneImageData)
{
    return QImageToCvMat(inPixmap.toImage(), inCloneImageData);
}

// If inImage exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inImage's
// data with the cv::Mat directly
//    NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
//    NOTE: This does not cover all cases - it should be easy to add new ones as required.
cv::Mat Kinect2BridgePrivate::QImageToCvMat(const QImage& inImage, bool inCloneImageData)
{
    switch (inImage.format())
    {
        // 8-bit, 4 channel
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
    {
        cv::Mat  mat(inImage.height(), inImage.width(),
            CV_8UC4,
            const_cast<uchar*>(inImage.bits()),
            static_cast<size_t>(inImage.bytesPerLine())
        );

        return (inCloneImageData ? mat.clone() : mat);
    }

    // 8-bit, 3 channel
    case QImage::Format_RGB32:
    {
        if (!inCloneImageData)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Conversion requires cloning so we don't modify the original QImage data");
        }

        cv::Mat  mat(inImage.height(), inImage.width(),
            CV_8UC4,
            const_cast<uchar*>(inImage.bits()),
            static_cast<size_t>(inImage.bytesPerLine())
        );

        cv::Mat  matNoAlpha;

        cv::cvtColor(mat, matNoAlpha, cv::COLOR_BGRA2BGR);   // drop the all-white alpha channel

        return matNoAlpha;
    }

    // 8-bit, 3 channel
    case QImage::Format_RGB888:
    {
        if (!inCloneImageData)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Conversion requires cloning so we don't modify the original QImage data");
        }

        QImage   swapped = inImage.rgbSwapped();

        return cv::Mat(swapped.height(), swapped.width(),
            CV_8UC3,
            const_cast<uchar*>(swapped.bits()),
            static_cast<size_t>(swapped.bytesPerLine())
        ).clone();
    }

    // 8-bit, 1 channel
    case QImage::Format_Indexed8:
    {
        cv::Mat  mat(inImage.height(), inImage.width(),
            CV_8UC1,
            const_cast<uchar*>(inImage.bits()),
            static_cast<size_t>(inImage.bytesPerLine())
        );

        return (inCloneImageData ? mat.clone() : mat);
    }

    default:
        ROS_WARN_STREAM_NAMED("kinect2_bridge", "QImage format not handled in switch:" << inImage.format());
        break;
    }

    return cv::Mat();
}

QImage Kinect2BridgePrivate::cvMatToQImage(const cv::Mat& inMat)
{
    switch (inMat.type())
    {
        // 8-bit, 4 channel
    case CV_8UC4:
    {
        QImage image(inMat.data,
            inMat.cols, inMat.rows,
            static_cast<int>(inMat.step),
            QImage::Format_ARGB32);

        return image;
    }

    // 8-bit, 3 channel
    case CV_8UC3:
    {
        QImage image(inMat.data,
            inMat.cols, inMat.rows,
            static_cast<int>(inMat.step),
            QImage::Format_RGB888);

        return image.rgbSwapped();
    }
    case CV_16UC1:
    {
        QImage image(inMat.data,
            inMat.cols, inMat.rows,
            static_cast<int>(inMat.step),
            QImage::Format_Grayscale16);

        return image;
    }
    // 8-bit, 1 channel
    case CV_8UC1:
    {
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
        QImage image(inMat.data,
            inMat.cols, inMat.rows,
            static_cast<int>(inMat.step),
            QImage::Format_Grayscale8);
#else
        static QVector<QRgb>  sColorTable;

        // only create our color table the first time
        if (sColorTable.isEmpty())
        {
            sColorTable.resize(256);

            for (int i = 0; i < 256; ++i)
            {
                sColorTable[i] = qRgb(i, i, i);
            }
        }

        QImage image(inMat.data,
            inMat.cols, inMat.rows,
            static_cast<int>(inMat.step),
            QImage::Format_Indexed8);

        image.setColorTable(sColorTable);
#endif

        return image;
    }

    default:
        ROS_WARN_STREAM_NAMED("kinect2_bridge", "cv::Mat image type not handled in switch:" << inMat.type());
        break;
    }

    return QImage();
}

QPixmap Kinect2BridgePrivate::cvMatToQPixmap(const cv::Mat& inMat)
{
    return QPixmap::fromImage(cvMatToQImage(inMat));
}
