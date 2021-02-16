#include "kinect2_bridge/kinect2_bridge_private_linux.h"

bool Kinect2BridgePrivate::locateDevice(const std::string& sensor)
{
    if (sensor.empty())
    {
        sensor = freenect2.getDefaultDeviceSerialNumber();
    }

    OUT_INFO("Kinect2 devices found: ");
    for (int i = 0; i < numOfDevs; ++i)
    {
        const std::string &s = freenect2.getDeviceSerialNumber(i);
        deviceFound = deviceFound || s == sensor;
        OUT_INFO("  " << i << ": " FG_CYAN << s << (s == sensor ? FG_YELLOW " (selected)" : "") << NO_COLOR);
    }

    if (!deviceFound)
    {
        delete packetPipeline;
        return false;
    }

    return deviceFound;
}

bool Kinect2BridgePrivate::openDevice(const std::string& sensor)
{
    device = freenect2.openDevice(sensor, packetPipeline);

    if (device == 0)
    {
        OUT_INFO("no device connected or failure opening the default one!");
        return false;
    }

    listenerColor = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color);
    listenerIrDepth = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    device->setColorFrameListener(listenerColor);
    device->setIrAndDepthFrameListener(listenerIrDepth);

    return true;
}

bool Kinect2BridgePrivate::startDevice(const std::string& sensor)
{
    if (!device->start())
    {
        delete listenerIrDepth;
        delete listenerColor;
        return false;
    }

    OUT_INFO("device serial: " FG_CYAN << sensor << NO_COLOR);
    OUT_INFO("device firmware: " FG_CYAN << device->getFirmwareVersion() << NO_COLOR);

    colorParams = device->getColorCameraParams();
    irParams = device->getIrCameraParams();

    return true;
}

bool Kinect2BridgePrivate::stopDevice(const std::string& sensor)
{
    if (!device->stop())
    {
        OUT_ERROR("could not stop device!");
        delete listenerIrDepth;
        delete listenerColor;
        return false;
    }

    return true;
}

bool Kinect2BridgePrivate::closeDevice(const int sensor, bool closeOnly)
{
    if (!closeOnly)
    {
        delete listenerIrDepth;
        delete listenerColor;
        delete registration;

        delete depthRegLowRes;
        delete depthRegHighRes;
    }

    if (!device->close())
    {
        return false;
    }

    return true;
}

void Kinect2BridgePrivate::createCameraParameters(const std::string& sensor)
{
    OUT_INFO("default ir camera parameters: ");
    OUT_INFO("fx: " FG_CYAN << irParams.fx << NO_COLOR ", fy: " FG_CYAN << irParams.fy << NO_COLOR ", cx: " FG_CYAN << irParams.cx << NO_COLOR ", cy: " FG_CYAN << irParams.cy << NO_COLOR);
    OUT_INFO("k1: " FG_CYAN << irParams.k1 << NO_COLOR ", k2: " FG_CYAN << irParams.k2 << NO_COLOR ", p1: " FG_CYAN << irParams.p1 << NO_COLOR ", p2: " FG_CYAN << irParams.p2 << NO_COLOR ", k3: " FG_CYAN << irParams.k3 << NO_COLOR);

    OUT_INFO("default color camera parameters: ");
    OUT_INFO("fx: " FG_CYAN << colorParams.fx << NO_COLOR ", fy: " FG_CYAN << colorParams.fy << NO_COLOR ", cx: " FG_CYAN << colorParams.cx << NO_COLOR ", cy: " FG_CYAN << colorParams.cy << NO_COLOR);

    cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
    cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
    cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
    cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
    cameraMatrixColor.at<double>(2, 2) = 1;

    cameraMatrixIr.at<double>(0, 0) = irParams.fx;
    cameraMatrixIr.at<double>(1, 1) = irParams.fy;
    cameraMatrixIr.at<double>(0, 2) = irParams.cx;
    cameraMatrixIr.at<double>(1, 2) = irParams.cy;
    cameraMatrixIr.at<double>(2, 2) = 1;

    distortionIr.at<double>(0, 0) = irParams.k1;
    distortionIr.at<double>(0, 1) = irParams.k2;
    distortionIr.at<double>(0, 2) = irParams.p1;
    distortionIr.at<double>(0, 3) = irParams.p2;
    distortionIr.at<double>(0, 4) = irParams.k3;
}

bool Kinect2BridgePrivate::initRegistration(const std::string &method, const int32_t device, const double maxDepth);
{
    DepthRegistration::Method reg;

    if(method == "default")
    {
        reg = DepthRegistration::DEFAULT;
    }
    else if(method == "cpu")
    {
    #ifdef DEPTH_REG_CPU
        reg = DepthRegistration::CPU;
    #else
        OUT_ERROR("CPU registration is not available!");
        return false;
    #endif
    }
    else if(method == "opencl")
    {
    #ifdef DEPTH_REG_OPENCL
        reg = DepthRegistration::OPENCL;
    #else
        OUT_ERROR("OpenCL registration is not available!");
        return false;
    #endif
    }
    else
    {
        OUT_ERROR("Unknown registration method: " << method);
        return false;
    }

    depthRegLowRes = DepthRegistration::New(reg);
    depthRegHighRes = DepthRegistration::New(reg);

    if(!depthRegLowRes->init(m_d->cameraMatrixLowRes, sizeLowRes, m_d->cameraMatrixDepth, sizeIr, m_d->distortionDepth, m_d->rotation, m_d-> translation, 0.5f, maxDepth, device) ||
            !depthRegHighRes->init(m_d->cameraMatrixColor, sizeColor, m_d->cameraMatrixDepth, sizeIr, m_d->distortionDepth, m_d->rotation, m_d->translation, 0.5f, maxDepth, device))
    {
        delete depthRegLowRes;
        delete depthRegHighRes;
        return false;
    }

    // TODO: registration with libfreenect 2 or Kinect SDK in m_d!
    registration = new libfreenect2::Registration(irParams, colorParams);

    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Registration component initialized.");

    return true;
}

bool Kinect2BridgePrivate::initPipeline(const std::string &method, const int32_t device)
{
    if(method == "default")
    {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        packetPipeline = new libfreenect2::CudaPacketPipeline(device);
#elif defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
        packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#elif defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
        packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        packetPipeline = new libfreenect2::CpuPacketPipeline();
#endif
    }
    else if(method == "cpu")
    {
        packetPipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(method == "cuda")
    {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        packetPipeline = new libfreenect2::CudaPacketPipeline(device);
#else
        OUT_ERROR("Cuda depth processing is not available!");
        return false;
#endif
    }
    else if(method == "opencl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#else
        OUT_ERROR("OpenCL depth processing is not available!");
        return false;
#endif
    }
    else if(method == "opengl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        OUT_ERROR("OpenGL depth processing is not available!");
        return false;
#endif
    }
    else if(method == "clkde")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        packetPipeline = new libfreenect2::OpenCLKdePacketPipeline(device);
#else
        OUT_ERROR("OpenCL depth processing is not available!");
        return false;
#endif
    }
    else if(method == "cudakde")
    {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        packetPipeline = new libfreenect2::CudaKdePacketPipeline(device);
#else
        OUT_ERROR("Cuda depth processing is not available!");
        return false;
#endif
    }
    else
    {
        OUT_ERROR("Unknown depth processing method: " << method);
        return false;
    }

    return true;
}

void Kinect2BridgePrivate::initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth)
{
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Setting freenect2 device configuration...");

    libfreenect2::Freenect2Device::Config config;
    config.EnableBilateralFilter = bilateral_filter;
    config.EnableEdgeAwareFilter = edge_aware_filter;
    config.MinDepth = minDepth;
    config.MaxDepth = maxDepth;
    device->setConfiguration(config);
}

bool Kinect2BridgePrivate::startStreams(bool isSubscribedColor, bool isSubscribedDepth)
{
    if (device->startStreams(isSubscribedColor, isSubscribedDepth))
        return true;

    return false;
}

void Kinect2BridgePrivate::receiveIrDepth()
{
    libfreenect2::FrameMap frames;
    cv::Mat depth, ir;
    std_msgs::Header header;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status = this->status;
    size_t frame;

    if (!receiveFrames(listenerIrDepth, frames))
    {
        lockIrDepth.unlock();
        return;
    }
    double now = ros::Time::now().toSec();

    header = createHeader(lastDepth, lastColor);

    libfreenect2::Frame *irFrame = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depthFrame = frames[libfreenect2::Frame::Depth];

    if (irFrame->status != 0 || depthFrame->status != 0)
    {
        listenerIrDepth->release(frames);
        lockIrDepth.unlock();
        running = false;
        OUT_ERROR("failure in depth packet processor from libfreenect2");
        return;
    }
    if (irFrame->format != libfreenect2::Frame::Float || depthFrame->format != libfreenect2::Frame::Float)
    {
        listenerIrDepth->release(frames);
        lockIrDepth.unlock();
        running = false;
        OUT_ERROR("received invalid frame format");
        return;
    }

    frame = frameIrDepth++;

    if (status[COLOR_SD_RECT] || status[DEPTH_SD] || status[DEPTH_SD_RECT] || status[DEPTH_QHD] || status[DEPTH_HD])
    {
        cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data).copyTo(depth);
    }

    if(status[IR_SD] || status[IR_SD_RECT])
    {
        ir = cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data);
        ir.convertTo(images[IR_SD], CV_16U);
    }

    listenerIrDepth->release(frames);
    lockIrDepth.unlock();

    processIrDepth(depth, images, status);

    publishImages(images, header, status, frame, pubFrameIrDepth, IR_SD, COLOR_HD);

    double elapsed = ros::Time::now().toSec() - now;
    lockTime.lock();
    elapsedTimeIrDepth += elapsed;
    lockTime.unlock();
}

void Kinect2BridgePrivate::receiveColor()
{
    libfreenect2::FrameMap frames;
    std_msgs::Header header;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status = this->status;
    size_t frame;

    if (!receiveFrames(listenerColor, frames))
    {
        lockColor.unlock();
        return;
    }
    double now = ros::Time::now().toSec();

    header = createHeader(lastColor, lastDepth);

    libfreenect2::Frame *colorFrame = frames[libfreenect2::Frame::Color];

    if (colorFrame->status != 0)
    {
        listenerColor->release(frames);
        lockIrDepth.unlock();
        running = false;
        OUT_ERROR("failure in rgb packet processor from libfreenect2");
        return;
    }
    if (colorFrame->format != libfreenect2::Frame::BGRX && colorFrame->format != libfreenect2::Frame::RGBX)
    {
        listenerColor->release(frames);
        lockIrDepth.unlock();
        running = false;
        OUT_ERROR("received invalid frame format");
        return;
    }

    frame = frameColor++;

    cv::Mat color = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC4, colorFrame->data);
    if (status[COLOR_SD_RECT])
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
        if (colorFrame->format == libfreenect2::Frame::BGRX)
        {
            cv::cvtColor(tmp, images[COLOR_HD], CV_BGRA2BGR);
        }
        else
        {
            cv::cvtColor(tmp, images[COLOR_HD], CV_RGBA2BGR);
        }
    }

    listenerColor->release(frames);
    lockColor.unlock();

    processColor(images, status);

    publishImages(images, header, status, frame, pubFrameColor, COLOR_HD, COUNT);

    double elapsed = ros::Time::now().toSec() - now;
    lockTime.lock();
    elapsedTimeColor += elapsed;
    lockTime.unlock();
}

bool Kinect2BridgePrivate::receiveFrames()
{
    bool newFrames = false;
    for(; !newFrames;)
    {
#ifdef LIBFREENECT2_THREADING_STDLIB
        newFrames = listener->waitForNewFrame(frames, 1000);
#else
        newFrames = true;
        listener->waitForNewFrame(frames);
#endif
        if (!deviceActive || !running || !ros::ok())
        {
            if(newFrames)
            {
                listener->release(frames);
            }
            return false;
        }
    }

    return newFrames;
}

void Kinect2BridgePrivate::processColorDepthFrame()
{
    cv::Mat tmp;
    libfreenect2::Frame depthFrame(sizeIr.width, sizeIr.height, 4, depth.data);
    libfreenect2::Frame undistorted(sizeIr.width, sizeIr.height, 4);
    libfreenect2::Frame registered(sizeIr.width, sizeIr.height, 4);
    lockRegSD.lock();
    registration->apply(&color, &depthFrame, &undistorted, &registered);
    lockRegSD.unlock();
    cv::flip(cv::Mat(sizeIr, CV_8UC4, registered.data), tmp, 1);
    if(color.format == libfreenect2::Frame::BGRX)
    {
        cv::cvtColor(tmp, images[COLOR_SD_RECT], CV_BGRA2BGR);
    }
    else
    {
        cv::cvtColor(tmp, images[COLOR_SD_RECT], CV_RGBA2BGR);
    }
}
