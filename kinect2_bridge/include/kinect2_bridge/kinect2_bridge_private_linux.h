class Kinect2BridgePrivate
{
public:
    bool initDevice(const std::string& sensor);
    bool shutdownDevice();

    bool startDevice(const std::string& sensor);
    bool stopDevice(const std::string& sensor);

    int getNumDevices();
    bool locateDevice(const std::string& sensor);

    bool openDevice(const std::string& sensor);
    bool closeDevice(const std::string& sensor, bool closeOnly = true);

    void createCameraParameters(const std::string& sensor);

    libfreenect2::Frame color;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *device;
    libfreenect2::SyncMultiFrameListener *listenerColor, *listenerIrDepth;
    libfreenect2::PacketPipeline *packetPipeline;
    libfreenect2::Registration *registration;
    libfreenect2::Freenect2Device::ColorCameraParams colorParams;
    libfreenect2::Freenect2Device::IrCameraParams irParams;

    cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr, cameraMatrixDepth, distortionDepth;
    cv::Mat rotation, translation;

    bool initRegistration(const std::string &method, const int32_t device, const double maxDepth);
    bool initPipeline(const std::string &method, const int32_t device);

    void initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth);

    bool startStreams(bool isSubscribedColor, bool isSubscribedDepth);

    void receiveIrDepth();
    void receiveColor();
    bool receiveFrames();

    bool receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames);

    void processColorDepthFrame();
};
