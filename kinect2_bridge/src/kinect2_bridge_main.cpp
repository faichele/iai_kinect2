#include "kinect2_bridge/kinect2_bridge.h"

#include <string>

void helpOption(const std::string &name, const std::string &stype, const std::string &value, const std::string &desc)
{
    std::cout << FG_GREEN "_" << name << NO_COLOR ":=" FG_YELLOW "<" << stype << ">" NO_COLOR << std::endl
              << "    default: " FG_CYAN << value << NO_COLOR << std::endl
              << "    info:    " << desc << std::endl;
}

void help(const std::string &path)
{
    std::string depthMethods = "cpu";
    std::string depthDefault = "cpu";
    std::string regMethods = "default";
    std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    depthMethods += ", opengl";
    depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    depthMethods += ", opencl";
    depthDefault = "opencl";
#endif
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    depthMethods += ", cuda";
    depthMethods += ", cudakde";
    depthDefault = "cuda";
#endif
#ifdef DEPTH_REG_CPU
    regMethods += ", cpu";
#endif
#ifdef DEPTH_REG_OPENCL
    regMethods += ", opencl";
    regMethods += ", clkde";
    regDefault = "opencl";
#endif

    std::cout << path << FG_BLUE " [_options:=value]" << std::endl;
    helpOption("base_name",         "string", K2_DEFAULT_NS,  "set base name for all topics");
    helpOption("sensor",            "double", "-1.0",         "serial of the sensor to use");
    helpOption("fps_limit",         "double", "-1.0",         "limit the frames per second");
    helpOption("calib_path",        "string", K2_CALIB_PATH,  "path to the calibration files");
    helpOption("use_png",           "bool",   "false",        "Use PNG compression instead of TIFF");
    helpOption("jpeg_quality",      "int",    "90",           "JPEG quality level from 0 to 100");
    helpOption("png_level",         "int",    "1",            "PNG compression level from 0 to 9");
    helpOption("depth_method",      "string", depthDefault,   "Use specific depth processing: " + depthMethods);
    helpOption("depth_device",      "int",    "-1",           "openCL device to use for depth processing");
    helpOption("reg_method",        "string", regDefault,     "Use specific depth registration: " + regMethods);
    helpOption("reg_device",        "int",    "-1",           "openCL device to use for depth registration");
    helpOption("max_depth",         "double", "12.0",         "max depth value");
    helpOption("min_depth",         "double", "0.1",          "min depth value");
    helpOption("queue_size",        "int",    "2",            "queue size of publisher");
    helpOption("bilateral_filter",  "bool",   "true",         "enable bilateral filtering of depth images");
    helpOption("edge_aware_filter", "bool",   "true",         "enable edge aware filtering of depth images");
    helpOption("publish_tf",        "bool",   "false",        "publish static tf transforms for camera");
    helpOption("base_name_tf",      "string", "as base_name", "base name for the tf frames");
    helpOption("worker_threads",    "int",    "4",            "number of threads used for processing the images");
}

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
    ROSCONSOLE_AUTOINIT;
    if(!getenv("ROSCONSOLE_FORMAT"))
    {
        ros::console::g_formatter.tokens_.clear();
        ros::console::g_formatter.init("[${severity}] ${message}");
    }
#endif

    ros::init(argc, argv, "kinect2_bridge", ros::init_options::AnonymousName);

    for(int argI = 1; argI < argc; ++argI)
    {
        std::string arg(argv[argI]);

        if(arg == "--help" || arg == "--h" || arg == "-h" || arg == "-?" || arg == "--?")
        {
            help(argv[0]);
            ros::shutdown();
            return 0;
        }
        else
        {
            OUT_ERROR("Unknown argument: " << arg);
            return -1;
        }
    }

    if (!ros::ok())
    {
        OUT_ERROR("ros::ok failed!");
        return -1;
    }

    Kinect2Bridge kinect2;
    if (kinect2.start())
    {
#ifdef _WIN32
        //if (lockIrDepth.try_lock() && lockColor.try_lock())
        ros::Rate mainRate(5.0);
        while (ros::ok())
        {
            // OUT_INFO("Successfully locked IR/depth and color locks.");
            /*if (kinect2.receiveFrames())
            {
                OUT_INFO("Successfully retrieved next frame set from Kinect2 sensor.");
            }
            else
            {
                OUT_ERROR("Failed to retrieve next frame set from Kinect2 sensor!");
            }*/

            mainRate.sleep();
            ::ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
        }

        //else
        //{
            // OUT_ERROR("Failed to lock IR/depth and color locks!");
        //}
#else
        ros::spin();
#endif

        kinect2.stop();
    }

    ros::shutdown();
    return 0;
}
