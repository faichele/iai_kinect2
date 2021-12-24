/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kinect2_bridge/kinect2_bridge.h"

#ifdef _WIN32
#include "kinect2_bridge/kinect2_bridge_private_windows.h"
#else
#include "kinect2_bridge/kinect2_bridge_private_linux.h"
#endif

Kinect2Bridge::ImagePublisherOption Kinect2Bridge::emptyImagePublisherOption = Kinect2Bridge::ImagePublisherOption();

Kinect2Bridge::Kinect2Bridge(bool readImages, const ros::NodeHandle& nh, const ros::NodeHandle& priv_nh)
    : sizeColor(1920, 1080), sizeIr(512, 424), sizeLowRes(sizeColor.width / 2, sizeColor.height / 2),
    // color(sizeColor.width, sizeColor.height, 4), nh(nh), priv_nh(priv_nh),
    frameColor(0), frameIrDepth(0), pubFrameColor(0), pubFrameIrDepth(0), lastColor(0, 0), lastDepth(0, 0), nextColor(false),
    nextIrDepth(false), depthShift(0), running(false), deviceActive(false), clientConnected(false),
    imagePubOptionsRetrieved(false), m_readImages(readImages)
{
    status.resize(COUNT, UNSUBCRIBED);

    keypoint_indices_to_joint_names_map[(int)JointType_SpineBase] = "SPINE_BASE";
    keypoint_indices_to_joint_names_map[(int)JointType_SpineMid] = "SPINE_MID";
    keypoint_indices_to_joint_names_map[(int)JointType_Neck] = "NECK";
    keypoint_indices_to_joint_names_map[(int)JointType_Head] = "HEAD";
    keypoint_indices_to_joint_names_map[(int)JointType_ShoulderLeft] = "SHOULDER_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_ElbowLeft] = "ELBOW_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_WristLeft] = "WRIST_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_HandLeft] = "HAND_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_ShoulderRight] = "SHOULDER_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_ElbowRight] = "ELBOW_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_WristRight] = "WRIST_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_HandRight] = "HAND_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_HipLeft] = "HIP_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_KneeLeft] = "KNEE_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_AnkleLeft] = "ANKLE_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_FootLeft] = "FOOT_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_HipRight] = "HIP_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_KneeRight] = "KNEE_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_AnkleRight] = "ANKLE_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_FootRight] = "FOOT_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_SpineShoulder] = "SPINE_SHOULDER";
    keypoint_indices_to_joint_names_map[(int)JointType_HandTipLeft] = "HAND_TIP_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_ThumbLeft] = "THUMB_LEFT";
    keypoint_indices_to_joint_names_map[(int)JointType_HandTipRight] = "HAND_TIP_RIGHT";
    keypoint_indices_to_joint_names_map[(int)JointType_ThumbRight] = "THUMB_RIGHT";

    bone_name_to_keypoint_indices_map["LowerSpine_0"] = std::make_pair<int, int>((int)JointType_SpineBase, (int)JointType_SpineMid);
    bone_name_to_keypoint_indices_map["UpperSpine_1"] = std::make_pair<int, int>((int)JointType_SpineMid, (int)JointType_SpineShoulder);
    bone_name_to_keypoint_indices_map["Neck_2"] = std::make_pair<int, int>((int)JointType_SpineShoulder, (int)JointType_Neck);
    bone_name_to_keypoint_indices_map["Head_3"] = std::make_pair<int, int>((int)JointType_Neck, (int)JointType_Head);

    bone_name_to_keypoint_indices_map["LeftShoulder_4"] = std::make_pair<int, int>((int)JointType_SpineShoulder, (int)JointType_ShoulderLeft);
    bone_name_to_keypoint_indices_map["LeftUpperArm_5"] = std::make_pair<int, int>((int)JointType_ShoulderLeft, (int)JointType_ElbowLeft);
    bone_name_to_keypoint_indices_map["LeftLowerArm_6"] = std::make_pair<int, int>((int)JointType_ElbowLeft, (int)JointType_WristLeft);
    bone_name_to_keypoint_indices_map["LeftWrist_7"] = std::make_pair<int, int>((int)JointType_WristLeft, (int)JointType_HandLeft);
    bone_name_to_keypoint_indices_map["LeftHandTip_8"] = std::make_pair<int, int>((int)JointType_HandLeft, (int)JointType_HandTipLeft);
    bone_name_to_keypoint_indices_map["LeftThumb_9"] = std::make_pair<int, int>((int)JointType_HandLeft, (int)JointType_ThumbLeft);

    bone_name_to_keypoint_indices_map["RightShoulder_10"] = std::make_pair<int, int>((int)JointType_SpineShoulder, (int)JointType_ShoulderRight);
    bone_name_to_keypoint_indices_map["RightUpperArm_11"] = std::make_pair<int, int>((int)JointType_ShoulderRight, (int)JointType_ElbowRight);
    bone_name_to_keypoint_indices_map["RightLowerArm_12"] = std::make_pair<int, int>((int)JointType_ElbowRight, (int)JointType_WristRight);
    bone_name_to_keypoint_indices_map["RightWrist_13"] = std::make_pair<int, int>((int)JointType_WristRight, (int)JointType_HandRight);
    bone_name_to_keypoint_indices_map["RightHandTip_14"] = std::make_pair<int, int>((int)JointType_HandRight, (int)JointType_HandTipRight);
    bone_name_to_keypoint_indices_map["RightThumb_15"] = std::make_pair<int, int>((int)JointType_HandRight, (int)JointType_ThumbRight);

    bone_name_to_keypoint_indices_map["LeftHip_16"] = std::make_pair<int, int>((int)JointType_SpineBase, (int)JointType_HipLeft);
    bone_name_to_keypoint_indices_map["LeftFemur_17"] = std::make_pair<int, int>((int)JointType_HipLeft, (int)JointType_KneeLeft);
    bone_name_to_keypoint_indices_map["LeftShank_18"] = std::make_pair<int, int>((int)JointType_KneeLeft, (int)JointType_AnkleLeft);
    bone_name_to_keypoint_indices_map["LeftFoot_19"] = std::make_pair<int, int>((int)JointType_AnkleLeft, (int)JointType_FootLeft);

    bone_name_to_keypoint_indices_map["RightHip_20"] = std::make_pair<int, int>((int)JointType_SpineBase, (int)JointType_HipRight);
    bone_name_to_keypoint_indices_map["RightFemur_21"] = std::make_pair<int, int>((int)JointType_HipRight, (int)JointType_KneeRight);
    bone_name_to_keypoint_indices_map["RightShank_22"] = std::make_pair<int, int>((int)JointType_KneeRight, (int)JointType_AnkleRight);
    bone_name_to_keypoint_indices_map["RightFoot_23"] = std::make_pair<int, int>((int)JointType_AnkleRight, (int)JointType_FootRight);
    
    m_d.reset(new Kinect2BridgePrivate(m_readImages));
}

Kinect2Bridge::~Kinect2Bridge()
{

}

bool Kinect2Bridge::start()
{
    if (running)
    {
        OUT_ERROR("kinect2_bridge is already running!");
        return false;
    }
    if (!initialize())
    {
        OUT_ERROR("Initialization failed!");
        return false;
    }

    OUT_INFO("Setting running flag to true in start().");
    running = true;

    if (publishTF)
    {
        OUT_INFO("Starting static TF publisher thread.");
        tfPublisher = std::thread(&Kinect2Bridge::publishStaticTF, this);
    }

#ifndef _WIN32
    for (size_t i = 0; i < threads.size(); ++i)
    {
        threads[i] = std::thread(&Kinect2Bridge::threadDispatcher, this, i);
    }
#endif

    mainThread = std::thread(&Kinect2Bridge::main, this);
    return true;
}

void Kinect2Bridge::stop()
{
    if (!running)
    {
        OUT_ERROR("kinect2_bridge is not running!");
        return;
    }

    OUT_INFO("Setting running flag to false in stop().");
    running = false;

    OUT_INFO("Waiting for mainThread to join...");
    mainThread.join();
    OUT_INFO("mainThread joined.");

#ifndef _WIN32
    for (size_t i = 0; i < threads.size(); ++i)
    {
        threads[i].join();
    }
#endif

    if (publishTF)
    {
        tfPublisher.join();
    }

    if (deviceActive && !m_d->stopDevice(kinect_sensor))
    {
        OUT_ERROR("could not stop device in stop()!");
    }

    if (!m_d->closeDevice(kinect_sensor))
    {
        OUT_ERROR("could not close device!");
    }

#ifdef _WIN32
    if (!m_d->shutdownDevice())
    {
        OUT_ERROR("could not shutdown device!");
    }
#endif

    for(std::map<Image, ros::Publisher>::iterator it = imagePubs.begin(); it != imagePubs.end(); ++it)
    {
        it->second.shutdown();
    }

    for (std::map<Image, ros::Publisher>::iterator it = compressedPubs.begin(); it != compressedPubs.end(); ++it)
    {
        it->second.shutdown();
    }

    infoHDPub.shutdown();
    infoQHDPub.shutdown();
    infoIRPub.shutdown();

    nh.shutdown();
}

Kinect2Bridge::Image Kinect2Bridge::stringToImageType(const std::string& imageType)
{
    if (imageType.compare("IR_SD") == 0)
        return IR_SD;
    if (imageType.compare("IR_SD_RECT") == 0)
        return IR_SD_RECT;
    if (imageType.compare("DEPTH_SD") == 0)
        return DEPTH_SD;
    if (imageType.compare("DEPTH_SD_RECT") == 0)
        return DEPTH_SD_RECT;
    if (imageType.compare("DEPTH_HD") == 0)
        return DEPTH_HD;
    if (imageType.compare("DEPTH_QHD") == 0)
        return DEPTH_QHD;
    if (imageType.compare("COLOR_SD_RECT") == 0)
        return COLOR_SD_RECT;
    if (imageType.compare("COLOR_HD") == 0)
        return COLOR_HD;
    if (imageType.compare("COLOR_HD_RECT") == 0)
        return COLOR_HD_RECT;
    if (imageType.compare("COLOR_QHD") == 0)
        return COLOR_QHD;
    if (imageType.compare("COLOR_QHD_RECT") == 0)
        return COLOR_QHD_RECT;
    if (imageType.compare("MONO_HD") == 0)
        return MONO_HD;
    if (imageType.compare("MONO_HD_RECT") == 0)
        return MONO_HD_RECT;
    if (imageType.compare("MONO_QHD") == 0)
        return MONO_QHD;
    if (imageType.compare("MONO_QHD_RECT") == 0)
        return MONO_QHD_RECT;

    return COUNT;
}

bool Kinect2Bridge::retrieveImagePubOptions()
{
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Retrieving image publisher options from ROS parameter server.");
    if (priv_nh.hasParam("/kinect2_bridge/image_publisher_options"))
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "/kinect2_bridge_image_publisher_options parameter found.");
        XmlRpc::XmlRpcValue img_pub_param_value;
        if (priv_nh.getParam("/kinect2_bridge/image_publisher_options", img_pub_param_value))
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "/kinect2_bridge/image_publisher_options parameter retrieved: " << img_pub_param_value);
            if (img_pub_param_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_INFO_STREAM_NAMED("kinect2_bridge", "Parameter type is XmlRpc::XmlRpcValue::TypeStruct as expected with " << img_pub_param_value.size() << " entries.");
                imagePubOptionsRetrieved = true;
                imagePublisherOptions.clear();

                for (XmlRpc::XmlRpcValue::ValueStruct::iterator it = img_pub_param_value.begin(); it != img_pub_param_value.end(); it++)
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "  Struct member: " << it->first << " of type " << it->second.getType());
                    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "  Struct entry of size: " << it->second.size());

                        ImagePublisherOption img_pub_option;
                        bool img_type_found = false;
                        bool publish_found = false;
                        bool publish_compressed_found = false;
                        std::string img_type_string;
                        bool img_type_definition_valid = false;

                        for (XmlRpc::XmlRpcValue::ValueStruct::iterator it_img = it->second.begin(); it_img != it->second.end(); it_img++)
                        {
                            if (it_img->first == "image_type")
                            {
                                ROS_INFO_STREAM_NAMED("kinect2_bridge", "   image_type value found: " << it_img->second);
                                img_pub_option.imageType = stringToImageType(it_img->second);
                                img_type_string = it_img->second;

                                if (img_pub_option.imageType != COUNT)
                                    img_type_found = true;
                            }

                            if (it_img->first == "publish")
                            {
                                ROS_INFO_STREAM_NAMED("kinect2_bridge", "   publish value found: " << it_img->second);
                                if (it_img->second.operator bool & () == true)
                                    img_pub_option.publish = true;
                                else
                                    img_pub_option.publish = false;
                                
                                publish_found = true;
                            }

                            if (it_img->first == "publish_compressed")
                            {
                                ROS_INFO_STREAM_NAMED("kinect2_bridge", "   publish_compressed value found: " << it_img->second);
                                if (it_img->second.operator bool & () == true)
                                    img_pub_option.publishCompressed = true;
                                else
                                    img_pub_option.publishCompressed = false;

                                publish_compressed_found = true;
                            }
                        }

                        if (publish_found && publish_compressed_found && img_type_found)
                        {
                            img_type_definition_valid = true;
                        }
                        else
                        {
                            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Invalid ImagePublisherOption encountered: " << it->first << "!");
                        }

                        if (img_type_definition_valid)
                        {
                            img_pub_option.valid = true;
                            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Adding new image type definition for image format: " << img_type_string);
                            imagePublisherOptions.push_back(img_pub_option);
                        }
                    }
                }

                return true;
            }
        }
    }
    return false;
}

bool Kinect2Bridge::retrieveColorAndIrParams()
{
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Retrieving color and IR parameter settings from ROS parameter server.");
    if (priv_nh.hasParam("/kinect2_bridge/colorParams") && priv_nh.hasParam("/kinect2_bridge/irParams"))
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Both /kinect2_bridge/colorParams and /kinect2_bridge/irParams parameters found.");
        XmlRpc::XmlRpcValue color_params_value;
        XmlRpc::XmlRpcValue ir_params_value;
        if (priv_nh.getParam("/kinect2_bridge/colorParams", color_params_value) && priv_nh.getParam("/kinect2_bridge/irParams", ir_params_value))
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "/kinect2_bridge/colorParams value retrieved: " << color_params_value);
            if (color_params_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_INFO_STREAM_NAMED("kinect2_bridge", "Parameter type is XmlRpc::XmlRpcValue::TypeStruct as expected with " << color_params_value.size() << " entries.");
                
                bool color_cx_found = false;
                double color_cx = 0.0f;
                bool color_cy_found = false;
                double color_cy = 0.0f;
                bool color_fx_found = false;
                double color_fx = 0.0f;
                bool color_fy_found = false;
                double color_fy = 0.0f;

                for (XmlRpc::XmlRpcValue::ValueStruct::iterator it = color_params_value.begin(); it != color_params_value.end(); it++)
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "  Struct member: " << it->first << " of type " << it->second.getType());
                    
                    if (it->first == "fx")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   fx parameter for colorParams found: " << it->second);
                        color_fx_found = true;
                        color_fx = it->second.operator const double& ();
                    }
                    if (it->first == "fy")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   fy parameter for colorParams found: " << it->second);
                        color_fy_found = true;
                        color_fy = it->second.operator const double& ();
                    }
                    if (it->first == "cx")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   cx parameter for colorParams found: " << it->second);
                        color_cx_found = true;
                        color_cx = it->second.operator const double& ();
                    }
                    if (it->first == "cy")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   cy parameter for colorParams found: " << it->second);
                        color_cy_found = true;
                        color_cy = it->second.operator const double& ();
                    }
                }

                if (color_cx_found && color_cy_found && color_fx_found && color_fy_found)
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Found all required color parameters to set for private implementation.");
                    m_d->setColorParams(color_cx, color_cy, color_fx, color_fy);
                }
            }
            
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "/kinect2_bridge/irParams value retrieved: " << ir_params_value);
            if (ir_params_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_INFO_STREAM_NAMED("kinect2_bridge", "Parameter type is XmlRpc::XmlRpcValue::TypeStruct as expected with " << ir_params_value.size() << " entries.");

                bool ir_cx_found = false;
                double ir_cx = 0.0f;
                bool ir_cy_found = false;
                double ir_cy = 0.0f;
                bool ir_fx_found = false;
                double ir_fx = 0.0f;
                bool ir_fy_found = false;
                double ir_fy = 0.0f;
                bool ir_k1_found = false;
                double ir_k1 = 0.0f;
                bool ir_k2_found = false;
                double ir_k2 = 0.0f;
                bool ir_k3_found = false;
                double ir_k3 = 0.0f;
                bool ir_p1_found = false;
                double ir_p1 = 0.0f;
                bool ir_p2_found = false;
                double ir_p2 = 0.0f;

                for (XmlRpc::XmlRpcValue::ValueStruct::iterator it = ir_params_value.begin(); it != ir_params_value.end(); it++)
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "  Struct member: " << it->first << " of type " << it->second.getType());
                    if (it->first == "fx")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   fx parameter for irParams found: " << it->second);
                        ir_fx_found = true;
                        ir_fx = it->second.operator const double& ();
                    }
                    if (it->first == "fy")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   fy parameter for irParams found: " << it->second);
                        ir_fy_found = true;
                        ir_fy = it->second.operator const double& ();
                    }
                    if (it->first == "cx")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   cx parameter for irParams found: " << it->second);
                        ir_cx_found = true;
                        ir_cx = it->second.operator const double& ();
                    }
                    if (it->first == "cy")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   cy parameter for irParams found: " << it->second);
                        ir_cy_found = true;
                        ir_cy = it->second.operator const double& ();
                    }
                    if (it->first == "k1")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   k1 parameter for irParams found: " << it->second);
                        ir_k1_found = true;
                        ir_k1 = it->second.operator const double& ();
                    }
                    if (it->first == "k2")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   k2 parameter for irParams found: " << it->second);
                        ir_k2_found = true;
                        ir_k2 = it->second.operator const double& ();
                    }
                    if (it->first == "k3")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   k3 parameter for irParams found: " << it->second);
                        ir_k3_found = true;
                        ir_k3 = it->second.operator const double& ();
                    }
                    if (it->first == "p1")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   p1 parameter for irParams found: " << it->second);
                        ir_p1_found = true;
                        ir_p1 = it->second.operator const double& ();
                    }
                    if (it->first == "p2")
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "   p2 parameter for irParams found: " << it->second);
                        ir_p2_found = true;
                        ir_p2 = it->second.operator const double& ();
                    }
                }

                if (ir_cx_found && ir_cy_found && ir_fx_found && ir_fy_found && ir_k1_found && ir_k2_found && ir_k3_found && ir_p1_found && ir_p2_found)
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Found all required IR parameters to set for private implementation.");
                    m_d->setIRParams(ir_cx, ir_cy, ir_fx, ir_fy, ir_k1, ir_k2, ir_k3, ir_p1, ir_p2);
                }
            }
            
            return true;
        }
    }
    else
    {
        ROS_WARN_STREAM_NAMED("kinect2_bridge", "Setting default color and IR parameters matching one specific Kinect2 device. This is not what you should use, see to provide your own device's settings!");

        double color_cx = 959.5;
        double color_cy = 539.5;
        double color_fx = 1081.37;
        double color_fy = 1081.37;

        m_d->setColorParams(color_cx, color_cy, color_fx, color_fy);

        double ir_cx = 254.3;
        double ir_cy = 203.045;
        double ir_fx = 366.781;
        double ir_fy = 366.781;
        double ir_k1 = 0.0857434;
        double ir_k2 = -0.270673;
        double ir_k3 = 0.102812;
        double ir_p1 = 0.0;
        double ir_p2 = 0.0;
        
        m_d->setIRParams(ir_cx, ir_cy, ir_fx, ir_fy, ir_k1, ir_k2, ir_k3, ir_p1, ir_p2);

        return true;
    }

    return false;
}

bool Kinect2Bridge::initialize()
{
    double fps_limit, maxDepth, minDepth;
    bool use_png, bilateral_filter, edge_aware_filter;
    int32_t jpeg_quality, png_level, queueSize, reg_dev, depth_dev, worker_threads;
    std::string depth_method, reg_method, calib_path, sensor, base_name;

    std::string depthDefault = "cpu";
    std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    depthDefault = "opencl";
#endif
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    depthDefault = "cuda";
#endif
#ifdef DEPTH_REG_OPENCL
    regDefault = "opencl";
#endif

    priv_nh.param("base_name", base_name, std::string(K2_DEFAULT_NS));
    priv_nh.param("sensor", sensor, std::string(""));
    priv_nh.param("fps_limit", fps_limit, -1.0);
    priv_nh.param("calib_path", calib_path, std::string(K2_CALIB_PATH));
    priv_nh.param("use_png", use_png, false);
    priv_nh.param("jpeg_quality", jpeg_quality, 90);
    priv_nh.param("png_level", png_level, 1);
    priv_nh.param("depth_method", depth_method, depthDefault);
    priv_nh.param("depth_device", depth_dev, -1);
    priv_nh.param("reg_method", reg_method, regDefault);
    priv_nh.param("reg_device", reg_dev, -1);
    priv_nh.param("max_depth", maxDepth, 12.0);
    priv_nh.param("min_depth", minDepth, 0.1);
    priv_nh.param("queue_size", queueSize, 2);
    priv_nh.param("bilateral_filter", bilateral_filter, true);
    priv_nh.param("edge_aware_filter", edge_aware_filter, true);
    priv_nh.param("publish_tf", publishTF, true);
    priv_nh.param("base_name_tf", baseNameTF, base_name);
    priv_nh.param("worker_threads", worker_threads, 4);

#ifndef _WIN32
    worker_threads = std::max(1, worker_threads);
#else
    // TODO: Multithreading where applicable for postprocessing for Win32
    worker_threads = 1;
#endif
    threads.resize(worker_threads);

    OUT_INFO("parameter:" << std::endl
             << "                  base_name: " FG_CYAN << base_name << NO_COLOR << std::endl
             << "                     sensor: " FG_CYAN << (sensor.empty() ? "default" : sensor) << NO_COLOR << std::endl
             << "                  fps_limit: " FG_CYAN << fps_limit << NO_COLOR << std::endl
             << "                 calib_path: " FG_CYAN << calib_path << NO_COLOR << std::endl
             << "                    use_png: " FG_CYAN << (use_png ? "true" : "false") << NO_COLOR << std::endl
             << "               jpeg_quality: " FG_CYAN << jpeg_quality << NO_COLOR << std::endl
             << "                  png_level: " FG_CYAN << png_level << NO_COLOR << std::endl
             << "               depth_method: " FG_CYAN << depth_method << NO_COLOR << std::endl
             << "               depth_device: " FG_CYAN << depth_dev << NO_COLOR << std::endl
             << "                 reg_method: " FG_CYAN << reg_method << NO_COLOR << std::endl
             << "                 reg_device: " FG_CYAN << reg_dev << NO_COLOR << std::endl
             << "                  max_depth: " FG_CYAN << maxDepth << NO_COLOR << std::endl
             << "                  min_depth: " FG_CYAN << minDepth << NO_COLOR << std::endl
             << "                 queue_size: " FG_CYAN << queueSize << NO_COLOR << std::endl
             << "           bilateral_filter: " FG_CYAN << (bilateral_filter ? "true" : "false") << NO_COLOR << std::endl
             << "          edge_aware_filter: " FG_CYAN << (edge_aware_filter ? "true" : "false") << NO_COLOR << std::endl
             << "                 publish_tf: " FG_CYAN << (publishTF ? "true" : "false") << NO_COLOR << std::endl
             << "               base_name_tf: " FG_CYAN << baseNameTF << NO_COLOR << std::endl
             << "             worker_threads: " FG_CYAN << worker_threads << NO_COLOR << std::endl
             );

    if (retrieveImagePubOptions())
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Retrieved image publishing settings from ROS parameter server.");
    }

#ifdef _WIN32
    if (retrieveColorAndIrParams())
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Retrieved color and IR camera parameters from ROS parameter server.");
        m_d->initRegistration("", 0, std::numeric_limits<double>::infinity());
    }
#endif

    deltaT = fps_limit > 0 ? (1.0 / fps_limit) : 0.0;

    if(calib_path.empty() || calib_path.back() != '/')
    {
        calib_path += '/';
    }

    initCompression(jpeg_quality, png_level, use_png);

    if(!initPipeline(depth_method, depth_dev))
    {
        return false;
    }

    if (!initDevice(sensor))
    {
        ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Failed to initialize Kinect2 device!");
        return false;
    }

    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Device initialized.");

    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Calling initConfig().");
    initConfig(bilateral_filter, edge_aware_filter, minDepth, maxDepth);
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Calling initCalibration().");
    initCalibration(calib_path, sensor);

    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Calling initRegistration().");
    if (!initRegistration(reg_method, reg_dev, maxDepth))
    {
        ROS_ERROR_STREAM_NAMED("kinect2_bridge", "Failed to initialize registration component!");

        if (!m_d->closeDevice(sensor, true))
        {
            OUT_ERROR("could not close device!");
        }
        return false;
    }

    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Calling createCameraInfo().");
    createCameraInfo();
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Calling initTopics().");
    initTopics(queueSize, base_name);

    return true;
}

bool Kinect2Bridge::initRegistration(const std::string &method, const int32_t device, const double maxDepth)
{
    if (!m_d->initRegistration(method, device, maxDepth))
    {
        OUT_ERROR("Failed to initialize depth registration!");
        return false;
    }

    return true;
}

bool Kinect2Bridge::initPipeline(const std::string &method, const int32_t device)
{
    if (!m_d->initPipeline(method, device))
    {
        OUT_ERROR("Failed to initialize Kinect2 pipeline!");
        return false;
    }

    return true;
}

void Kinect2Bridge::initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth)
{
    m_d->initConfig(bilateral_filter, edge_aware_filter, minDepth, maxDepth);
}

// Built against a newer OpenCV version under Windows with ROS noetic
void Kinect2Bridge::initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png)
{
    compressionParams.resize(7, 0);
#ifdef _WIN32
    compressionParams[0] = cv::IMWRITE_JPEG_QUALITY;
#else
    compressionParams[0] = CV_IMWRITE_JPEG_QUALITY;
#endif
    compressionParams[1] = jpegQuality;
#ifdef _WIN32
    compressionParams[2] = cv::IMWRITE_PNG_COMPRESSION;
#else
        compressionParams[2] = CV_IMWRITE_PNG_COMPRESSION;
#endif
    compressionParams[3] = pngLevel;
#ifdef _WIN32
    compressionParams[4] = cv::IMWRITE_PNG_STRATEGY;
    compressionParams[5] = cv::IMWRITE_PNG_STRATEGY_RLE;
#else
    compressionParams[4] = CV_IMWRITE_PNG_STRATEGY;
    compressionParams[5] = CV_IMWRITE_PNG_STRATEGY_RLE;
#endif
    compressionParams[6] = 0;

    if(use_png)
    {
        compression16BitExt = ".png";
        compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; png compressed";
    }
    else
    {
        compression16BitExt = ".tif";
        compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; tiff compressed";
    }
}

const Kinect2Bridge::ImagePublisherOption& Kinect2Bridge::getImagePublisherOption(const Image imageType)
{
    for (size_t k = 0; k < imagePublisherOptions.size(); ++k)
    {
        if (imagePublisherOptions[k].imageType == imageType)
            return imagePublisherOptions[k];
    }

    return emptyImagePublisherOption;
}

void Kinect2Bridge::initTopics(const int32_t queueSize, const std::string &base_name)
{
    imageTopics.resize(COUNT);
    publishCompressedImages.resize(COUNT);
    
    const ImagePublisherOption& ir_sd_opt = getImagePublisherOption(IR_SD);
    if (ir_sd_opt.valid && ir_sd_opt.publish)
        imageTopics[IR_SD] = K2_TOPIC_SD K2_TOPIC_IMAGE_IR;
    else
        imageTopics[IR_SD] = "";

    publishCompressedImages[IR_SD] = ir_sd_opt.publishCompressed;

    const ImagePublisherOption& ir_sd_rect_opt = getImagePublisherOption(IR_SD_RECT);
    if (ir_sd_rect_opt.valid && ir_sd_rect_opt.publish)
        imageTopics[IR_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[IR_SD_RECT] = "";

    publishCompressedImages[IR_SD_RECT] = ir_sd_rect_opt.publishCompressed;

    const ImagePublisherOption& depth_sd_opt = getImagePublisherOption(DEPTH_SD);
    if (depth_sd_opt.valid && depth_sd_opt.publish)
        imageTopics[DEPTH_SD] = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH;
    else
        imageTopics[DEPTH_SD] = "";

    publishCompressedImages[DEPTH_SD] = depth_sd_opt.publishCompressed;

    const ImagePublisherOption& depth_sd_rect_opt = getImagePublisherOption(DEPTH_SD_RECT);
    if (depth_sd_rect_opt.valid && depth_sd_rect_opt.publish)
        imageTopics[DEPTH_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[DEPTH_SD_RECT] = "";

    publishCompressedImages[DEPTH_SD_RECT] = depth_sd_rect_opt.publishCompressed;

    const ImagePublisherOption& depth_hd_opt = getImagePublisherOption(DEPTH_HD);
    if (depth_hd_opt.valid && depth_hd_opt.publish)
        imageTopics[DEPTH_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[DEPTH_HD] = "";

    publishCompressedImages[DEPTH_HD] = depth_hd_opt.publishCompressed;

    const ImagePublisherOption& depth_qhd_opt = getImagePublisherOption(DEPTH_QHD);
    if (depth_qhd_opt.valid && depth_qhd_opt.publish)
        imageTopics[DEPTH_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[DEPTH_QHD] = "";

    publishCompressedImages[DEPTH_QHD] = depth_qhd_opt.publishCompressed;

    const ImagePublisherOption& color_sd_rect_opt = getImagePublisherOption(COLOR_SD_RECT);
    if (color_sd_rect_opt.valid && color_sd_rect_opt.publish)
        imageTopics[COLOR_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[COLOR_SD_RECT] = "";

    publishCompressedImages[COLOR_SD_RECT] = color_sd_rect_opt.publishCompressed;

    const ImagePublisherOption& color_hd_opt = getImagePublisherOption(COLOR_HD);
    if (color_hd_opt.valid && color_hd_opt.publish)
        imageTopics[COLOR_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR;
    else
        imageTopics[COLOR_HD] = "";

    publishCompressedImages[COLOR_HD] = color_hd_opt.publishCompressed;

    const ImagePublisherOption& color_hd_rect_opt = getImagePublisherOption(COLOR_HD_RECT);
    if (color_hd_rect_opt.valid && color_hd_rect_opt.publish)
        imageTopics[COLOR_HD_RECT] = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[COLOR_HD_RECT] = "";

    publishCompressedImages[COLOR_HD_RECT] = color_hd_rect_opt.publishCompressed;

    const ImagePublisherOption& color_qhd_opt = getImagePublisherOption(COLOR_QHD);
    if (color_qhd_opt.valid && color_qhd_opt.publish)
        imageTopics[COLOR_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR;
    else
        imageTopics[COLOR_QHD] = "";

    publishCompressedImages[COLOR_QHD] = color_qhd_opt.publishCompressed;

    const ImagePublisherOption& color_qhd_rect_opt = getImagePublisherOption(COLOR_QHD_RECT);
    if (color_qhd_rect_opt.valid && color_qhd_rect_opt.publish)
        imageTopics[COLOR_QHD_RECT] = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[COLOR_QHD_RECT] = "";

    publishCompressedImages[COLOR_QHD_RECT] = color_qhd_rect_opt.publishCompressed;

    const ImagePublisherOption& mono_hd_opt = getImagePublisherOption(MONO_HD);
    if (mono_hd_opt.valid && mono_hd_opt.publish)
        imageTopics[MONO_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_MONO;
    else
        imageTopics[MONO_HD] = "";

    publishCompressedImages[MONO_HD] = mono_hd_opt.publishCompressed;

    const ImagePublisherOption& mono_hd_rect_opt = getImagePublisherOption(MONO_HD_RECT);
    if (mono_hd_rect_opt.valid && mono_hd_rect_opt.publish)
        imageTopics[MONO_HD_RECT] = K2_TOPIC_HD K2_TOPIC_IMAGE_MONO K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[MONO_HD_RECT] = "";

    publishCompressedImages[MONO_HD_RECT] = mono_hd_rect_opt.publishCompressed;

    const ImagePublisherOption& mono_qhd_opt = getImagePublisherOption(MONO_QHD);
    if (mono_qhd_opt.valid && mono_qhd_opt.publish)
        imageTopics[MONO_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_MONO;
    else
        imageTopics[MONO_QHD] = "";

    publishCompressedImages[MONO_QHD] = mono_qhd_opt.publishCompressed;

    const ImagePublisherOption& mono_qhd_rect_opt = getImagePublisherOption(MONO_QHD_RECT);
    if (mono_qhd_rect_opt.valid && mono_qhd_rect_opt.publish)
        imageTopics[MONO_QHD_RECT] = K2_TOPIC_QHD K2_TOPIC_IMAGE_MONO K2_TOPIC_IMAGE_RECT;
    else
        imageTopics[MONO_QHD_RECT] = "";

    publishCompressedImages[MONO_QHD_RECT] = mono_qhd_rect_opt.publishCompressed;

    //imagePubs.resize(COUNT);
    //compressedPubs.resize(COUNT);
    ros::SubscriberStatusCallback cb = boost::bind(&Kinect2Bridge::callbackStatus, this);

    for(size_t i = 0; i < COUNT; ++i)
    {
        if (!imageTopics[i].empty())
            imagePubs[(Image) i] = nh.advertise<sensor_msgs::Image>(base_name + imageTopics[i], queueSize, cb, cb);

        if (publishCompressedImages[(Image) i])
            compressedPubs[(Image) i] = nh.advertise<sensor_msgs::CompressedImage>(base_name + imageTopics[i] + K2_TOPIC_COMPRESSED, queueSize, cb, cb);
    }

    infoHDPub = nh.advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_HD + K2_TOPIC_INFO, queueSize, cb, cb);
    infoQHDPub = nh.advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_QHD + K2_TOPIC_INFO, queueSize, cb, cb);
    infoIRPub = nh.advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_SD + K2_TOPIC_INFO, queueSize, cb, cb);
}

bool Kinect2Bridge::initDevice(const std::string &sensor)
{
    bool deviceFound = false;

    kinect_sensor = sensor;
#ifdef _WIN32
    if (!m_d->initDevice(sensor))
    {
        OUT_ERROR("Default Kinect2 device not found!");
        return false;
    }
    else
    {
        OUT_INFO("Default Kinect2 device found.");
    }
#endif

    const int numOfDevs = m_d->getNumDevices();

    if (numOfDevs <= 0)
    {
        OUT_ERROR("no Kinect2 devices found!");
        return false;
    }

    deviceFound = m_d->locateDevice(sensor);

    if (!deviceFound)
    {
        OUT_ERROR("Device '" << sensor << "' not found!");
        return false;
    }

    if (!m_d->openDevice(sensor))
    {
        OUT_ERROR("Device '" << sensor << "' could not be opened!");
        return false;
    }

    OUT_INFO("starting kinect2");
    if (!m_d->startDevice(sensor))
    {
        OUT_ERROR("could not start device!");
        return false;
    }

    if (!m_d->stopDevice(sensor))
    {
        OUT_ERROR("could not stop device (in initDevice())!");
        return false;
    }

    m_d->cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
    m_d->distortionColor = cv::Mat::zeros(1, 5, CV_64F);

    m_d->cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
    m_d->distortionIr = cv::Mat::zeros(1, 5, CV_64F);

    m_d->createCameraParameters(sensor);

    m_d->cameraMatrixDepth = m_d->cameraMatrixIr.clone();
    m_d->distortionDepth = m_d->distortionIr.clone();

    m_d->rotation = cv::Mat::eye(3, 3, CV_64F);
    m_d->translation = cv::Mat::zeros(3, 1, CV_64F);

    return true;
}

void Kinect2Bridge::initCalibration(const std::string &calib_path, const std::string &sensor)
{
    std::string calibPath = calib_path + sensor + '/';

    struct stat fileStat;
    bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 ||
        #ifdef _WIN32
            !(fileStat.st_mode & S_IFDIR)
        #else
            !S_ISDIR(fileStat.st_mode)
        #endif
            ;

    if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, m_d->cameraMatrixColor, m_d->distortionColor))
    {
        OUT_WARN("using sensor defaults for color intrinsic parameters.");
    }

    if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, m_d->cameraMatrixDepth, m_d->distortionDepth))
    {
        OUT_WARN("using sensor defaults for ir intrinsic parameters.");
    }

    if(calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, m_d->rotation, m_d->translation))
    {
        OUT_WARN("using defaults for rotation and translation.");
    }

    if(calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
    {
        OUT_WARN("using defaults for depth shift.");
        depthShift = 0.0;
    }

    m_d->cameraMatrixLowRes = m_d->cameraMatrixColor.clone();
    m_d->cameraMatrixLowRes.at<double>(0, 0) /= 2;
    m_d->cameraMatrixLowRes.at<double>(1, 1) /= 2;
    m_d->cameraMatrixLowRes.at<double>(0, 2) /= 2;
    m_d->cameraMatrixLowRes.at<double>(1, 2) /= 2;

    const int mapType = CV_16SC2;
    cv::initUndistortRectifyMap(m_d->cameraMatrixColor, m_d->distortionColor, cv::Mat(), m_d->cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
    cv::initUndistortRectifyMap(m_d->cameraMatrixIr, m_d->distortionIr, cv::Mat(), m_d->cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
    cv::initUndistortRectifyMap(m_d->cameraMatrixColor, m_d->distortionColor, cv::Mat(), m_d->cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);

    OUT_INFO("camera parameters used:");
    OUT_INFO("camera matrix color:" FG_CYAN << std::endl << m_d->cameraMatrixColor << NO_COLOR);
    OUT_INFO("distortion coefficients color:" FG_CYAN << std::endl << m_d->distortionColor << NO_COLOR);
    OUT_INFO("camera matrix ir:" FG_CYAN << std::endl << m_d->cameraMatrixIr << NO_COLOR);
    OUT_INFO("distortion coefficients ir:" FG_CYAN << std::endl << m_d->distortionIr << NO_COLOR);
    OUT_INFO("camera matrix depth:" FG_CYAN << std::endl << m_d->cameraMatrixDepth << NO_COLOR);
    OUT_INFO("distortion coefficients depth:" FG_CYAN << std::endl << m_d->distortionDepth << NO_COLOR);
    OUT_INFO("rotation:" FG_CYAN << std::endl << m_d->rotation << NO_COLOR);
    OUT_INFO("translation:" FG_CYAN << std::endl << m_d->translation << NO_COLOR);
    OUT_INFO("depth shift:" FG_CYAN << std::endl << depthShift << NO_COLOR);
}

bool Kinect2Bridge::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
{
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
        fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
        fs[K2_CALIB_DISTORTION] >> distortion;
        fs.release();
    }
    else
    {
        OUT_ERROR("can't open calibration file: " << filename);
        return false;
    }
    return true;
}

bool Kinect2Bridge::loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
{
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
        fs[K2_CALIB_ROTATION] >> rotation;
        fs[K2_CALIB_TRANSLATION] >> translation;
        fs.release();
    }
    else
    {
        OUT_ERROR("can't open calibration pose file: " << filename);
        return false;
    }
    return true;
}

bool Kinect2Bridge::loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
{
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
        fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
        fs.release();
    }
    else
    {
        OUT_ERROR("can't open calibration depth file: " << filename);
        return false;
    }
    return true;
}

void Kinect2Bridge::createCameraInfo()
{
    cv::Mat projColor = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projIr = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projLowRes = cv::Mat::zeros(3, 4, CV_64F);

    m_d->cameraMatrixColor.copyTo(projColor(cv::Rect(0, 0, 3, 3)));
    m_d->cameraMatrixIr.copyTo(projIr(cv::Rect(0, 0, 3, 3)));
    m_d->cameraMatrixLowRes.copyTo(projLowRes(cv::Rect(0, 0, 3, 3)));

    createCameraInfo(sizeColor, m_d->cameraMatrixColor, m_d->distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor, infoHD);
    createCameraInfo(sizeIr, m_d->cameraMatrixIr, m_d->distortionIr, cv::Mat::eye(3, 3, CV_64F), projIr, infoIR);
    createCameraInfo(sizeLowRes, m_d->cameraMatrixLowRes, m_d->distortionColor, cv::Mat::eye(3, 3, CV_64F), projLowRes, infoQHD);
}

void Kinect2Bridge::createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const
{
    cameraInfo.height = size.height;
    cameraInfo.width = size.width;

    const double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
        cameraInfo.K[i] = *itC;
    }

    const double *itR = rotation.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itR)
    {
        cameraInfo.R[i] = *itR;
    }

    const double *itP = projection.ptr<double>(0, 0);
    for(size_t i = 0; i < 12; ++i, ++itP)
    {
        cameraInfo.P[i] = *itP;
    }

    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.D.resize(distortion.cols);
    const double *itD = distortion.ptr<double>(0, 0);
    for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
    {
        cameraInfo.D[i] = *itD;
    }
}

void Kinect2Bridge::callbackStatus()
{
    bool isSubscribedDepth = false;
    bool isSubscribedColor = false;

    lockStatus.lock();
    clientConnected = updateStatus(isSubscribedColor, isSubscribedDepth);
    bool error = false;

    if (clientConnected && !deviceActive)
    {
        OUT_INFO("Client connected. Starting device...");
        if (!m_d->startStreams(isSubscribedColor, isSubscribedDepth))
        {
            OUT_ERROR("Could not start device!");
            error = true;
        }
        else
        {
            OUT_INFO("Client connected, device started.");
            deviceActive = true;
        }
    }
    else if (!clientConnected && deviceActive)
    {
        OUT_INFO("No clients connected. Stopping device...");

        if (!m_d->stopDevice(kinect_sensor))
        {
            OUT_ERROR("Could not stop device (in callbackStatus(), 1)!");
            error = true;
        }
        else
        {
            OUT_INFO("No clients connected. Stopped device in callbackStatus().");
            deviceActive = false;
        }

    }
    else if (deviceActive && (isSubscribedColor != this->isSubscribedColor || isSubscribedDepth != this->isSubscribedDepth))
    {
        if (!m_d->stopDevice(kinect_sensor))
        {
            OUT_ERROR("could not stop device! (in callbackStatus(), 2)");
            error = true;
        }
        else if (!m_d->startStreams(isSubscribedColor, isSubscribedDepth))
        {
            OUT_ERROR("could not start device!");
            error = true;
            deviceActive = false;
        }
    }
    this->isSubscribedColor = isSubscribedColor;
    this->isSubscribedDepth = isSubscribedDepth;
    lockStatus.unlock();

    if (error)
    {
        stop();
    }
}

bool Kinect2Bridge::updateStatus(bool &isSubscribedColor, bool &isSubscribedDepth)
{
    isSubscribedDepth = false;
    isSubscribedColor = false;

    for (size_t i = 0; i < COUNT; ++i)
    {
        Status s = UNSUBCRIBED;
        if (imagePubs.find((Image) i) != imagePubs.end() && imagePubs[(Image) i].getNumSubscribers() > 0)
        {
            s = RAW;
        }
        if(compressedPubs.find((Image) i) != compressedPubs.end() && compressedPubs[(Image) i].getNumSubscribers() > 0)
        {
            s = s == RAW ? BOTH : COMPRESSED;
        }

        if (i <= COLOR_SD_RECT && s != UNSUBCRIBED)
        {
            isSubscribedDepth = true;
        }
        if (i >= COLOR_SD_RECT && s != UNSUBCRIBED)
        {
            isSubscribedColor = true;
        }

        status[i] = s;
    }

    if (infoHDPub.getNumSubscribers() > 0 || infoQHDPub.getNumSubscribers() > 0)
    {
        isSubscribedColor = true;
    }
    if (infoIRPub.getNumSubscribers() > 0)
    {
        isSubscribedDepth = true;
    }

    return isSubscribedColor || isSubscribedDepth;
}

void Kinect2Bridge::main()
{
    setThreadName("Controll");
    OUT_INFO("waiting for clients to connect");
    double nextFrame = ros::Time::now().toSec() + deltaT;
    double fpsTime = ros::Time::now().toSec();
    size_t oldFrameIrDepth = 0, oldFrameColor = 0;
    nextColor = true;
    nextIrDepth = true;

#ifdef _WIN32
    if (m_readImages)
    {
        running = true;
        if (m_d->readImagesFromDirectory("E:\\Temp"))
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Read input images from directory for offline processing.");
        }
    }
#endif

    ros::Rate queryRate(16.0);
    for (; running && ros::ok();)
    {
#ifndef _WIN32
        if (!deviceActive)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            fpsTime =  ros::Time::now().toSec();
            nextFrame = fpsTime + deltaT;
            continue;
        }
#endif

#ifdef _WIN32
        if (m_d->m_readImages)
        {
            m_d->publishNextImageSet();
            queryRate.sleep();
        }
        else
        {
            if (!m_d->receiveFrames())
            {
                ROS_WARN_STREAM_NAMED("kinect2_bridge", "Failed to query new frame data in control thread!");
                queryRate.sleep();
            }
            else
            {
                ROS_INFO_STREAM_NAMED("kinect2_bridge", "Iterating control thread. deviceActive = " << (int)deviceActive);

                std_msgs::Header msg_header = createHeader(lastColor, lastDepth);

                if (m_d->color_img_received)
                {
                    std::vector<cv::Mat> color_images(COLOR_HD + 1);
                    color_images[COLOR_HD] = m_d->last_color_img;

                    processColor(color_images, status);

                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Calling publishImages() for most current color frame...");
                    publishImages(color_images, msg_header, status, 0, pubFrameColor, COLOR_HD, COLOR_HD + 1);
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "publishImages() for most current color frame done.");
                    m_d->color_img_received = false;
                }

                if (m_d->depth_img_received)
                {
                    std::vector<cv::Mat> depth_ir_images(DEPTH_SD + 1);
                    depth_ir_images[IR_SD] = m_d->last_infrared_img;
                    depth_ir_images[DEPTH_SD] = m_d->last_depth_img;

                    processIrDepth(m_d->last_depth_img, depth_ir_images, this->status);

                    publishImages(depth_ir_images, msg_header, status, 0, pubFrameIrDepth, IR_SD, DEPTH_SD + 1);
                }

                if (m_d->body_frames_received)
                {
                    bb_persons_msgs::TrackingStates tracking_states_msg;
                    tracking_states_msg.tracking_states.resize(Kinect2BridgePrivate::cMaxTrackedUsers);
                    for (size_t k = 0; k < Kinect2BridgePrivate::cMaxTrackedUsers; k++)
                        tracking_states_msg.tracking_states[k] = m_d->m_userTrackingStatus[k];

                    m_d->m_trackingStatesPub.publish(tracking_states_msg);

                    bb_persons_msgs::Persons persons_msg;
                    for (std::map<unsigned int, Kinect2SkeletonData>::const_iterator it = m_d->m_trackedUsers.begin(); it != m_d->m_trackedUsers.end(); it++)
                    {
                        bb_persons_msgs::Person person_msg;
                        person_msg.tracking_id = it->second.trackingId;
                        person_msg.tracking_index = it->second.trackingIndex;

                        /*person_msg.tracking_states.tracking_states.resize(Kinect2BridgePrivate::cMaxTrackedUsers);
                        for (size_t k = 0; k < Kinect2BridgePrivate::cMaxTrackedUsers; k++)
                            person_msg.tracking_states.tracking_states[k] = m_d->m_userTrackingStatus[k];*/

                        //person_msg.header.stamp = ros::Time::now();
                        //person_msg.header.frame_id = "kinect2_link";

                        for (int k = 0; k < JointType_Count; k++)
                        {
                            geometry_msgs::Point point_k;
                            point_k.x = it->second.jointPositions3D[k].x();
                            point_k.y = it->second.jointPositions3D[k].y();
                            point_k.z = it->second.jointPositions3D[k].z();
                            person_msg.joints3D.push_back(point_k);

                            geometry_msgs::Point point_k_2d;
                            point_k_2d.x = it->second.jointPositions2D[k].x();
                            point_k_2d.y = it->second.jointPositions2D[k].y();
                            point_k_2d.z = 0.0;
                            person_msg.joints2D.push_back(point_k_2d);
                        }

                        persons_msg.persons.push_back(person_msg);
                    }

                    m_d->m_bodyFramesPub.publish(persons_msg);
                }

                queryRate.sleep();
            }
        }
#endif

#ifndef _WIN32
        double now = ros::Time::now().toSec();

        if (now - fpsTime >= 3.0)
        {
            fpsTime = now - fpsTime;
            size_t framesIrDepth = frameIrDepth - oldFrameIrDepth;
            size_t framesColor = frameColor - oldFrameColor;
            oldFrameIrDepth = frameIrDepth;
            oldFrameColor = frameColor;

            lockTime.lock();
            double tColor = elapsedTimeColor;
            double tDepth = elapsedTimeIrDepth;
            elapsedTimeColor = 0;
            elapsedTimeIrDepth = 0;
            lockTime.unlock();

            if (isSubscribedDepth)
            {
                OUT_INFO("depth processing: " FG_YELLOW "~" << (tDepth / framesIrDepth) * 1000 << "ms" NO_COLOR " (~" << framesIrDepth / tDepth << "Hz) publishing rate: " FG_YELLOW "~" << framesIrDepth / fpsTime << "Hz" NO_COLOR);
            }
            if (isSubscribedColor)
            {
                OUT_INFO("color processing: " FG_YELLOW "~" << (tColor / framesColor) * 1000 << "ms" NO_COLOR " (~" << framesColor / tColor << "Hz) publishing rate: " FG_YELLOW "~" << framesColor / fpsTime << "Hz" NO_COLOR);
            }
            fpsTime = now;
        }

        if (now >= nextFrame)
        {
            nextColor = true;
            nextIrDepth = true;
            nextFrame += deltaT;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!deviceActive)
        {
            oldFrameIrDepth = frameIrDepth;
            oldFrameColor = frameColor;
            lockTime.lock();
            elapsedTimeColor = 0;
            elapsedTimeIrDepth = 0;
            lockTime.unlock();
            continue;
        }
#endif
    }
}

void Kinect2Bridge::threadDispatcher(const size_t id)
{
    std::string workerThreadName("Worker" + std::to_string(id));
    setThreadName(workerThreadName);
    const size_t checkFirst = id % 2;
    bool processedFrame = false;
#ifdef _WIN32
    int oldNice = 0;
    ros::Rate queryRate(8.0);
#else
    int oldNice = nice(0);
    oldNice = nice(19 - oldNice);
#endif
    for(; running && ros::ok();)
    {
        processedFrame = false;
        ROS_INFO_STREAM_THROTTLE_NAMED(1.0, "kinect2_bridge", "threadDispacher() " << workerThreadName);
#ifndef _WIN32
        for (size_t i = 0; i < 2; ++i)
        {
            if (i == checkFirst)
            {
                if (nextIrDepth && lockIrDepth.try_lock())
                {
                    nextIrDepth = false;
                    ROS_INFO_STREAM_THROTTLE_NAMED(1.0, "kinect2_bridge", "Calling receiveIrDepth().");
                    receiveIrDepth();
                    processedFrame = true;
                }
            }
            else
            {
                if (nextColor && lockColor.try_lock())
                {
                    nextColor = false;
                    ROS_INFO_STREAM_THROTTLE_NAMED(1.0, "kinect2_bridge", "Calling receiveColor().");
                    receiveColor();
                    processedFrame = true;
                }
            }
        }
#endif

#ifndef _WIN32
        if (!processedFrame)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
#endif
    }
}

void Kinect2Bridge::receiveIrDepth()
{
#ifndef _WIN32
    m_d->receiveIrDepth();
#endif
}

void Kinect2Bridge::receiveColor()
{
#ifndef _WIN32
    m_d->receiveColor();
#endif
}

bool Kinect2Bridge::receiveFrames()
{
    ROS_INFO_STREAM_THROTTLE_NAMED(1.0, "kinect2_bridge", "Calling receiveFrames()");
    bool newFrames = m_d->receiveFrames();
    return newFrames;
}

std_msgs::Header Kinect2Bridge::createHeader(ros::Time &last, ros::Time &other)
{
    ros::Time timestamp = ros::Time::now();
    lockSync.lock();
    if (other.isZero())
    {
        last = timestamp;
    }
    else
    {
        timestamp = other;
        other = ros::Time(0, 0);
    }
    lockSync.unlock();

    std_msgs::Header header;
    header.seq = 0;
    header.stamp = timestamp;
    return header;
}

void Kinect2Bridge::processIrDepth(const cv::Mat &depth, std::vector<cv::Mat> &images, const std::vector<Kinect2Bridge::Status> &status)
{
    // COLOR registered to depth
    if (status[COLOR_SD_RECT])
    {
        m_d->processColorDepthFrame();
    }

    // IR
    if (status[IR_SD] || status[IR_SD_RECT])
    {
        cv::flip(images[IR_SD], images[IR_SD], 1);
    }
    if (status[IR_SD_RECT])
    {
        cv::remap(images[IR_SD], images[IR_SD_RECT], map1Ir, map2Ir, cv::INTER_AREA);
    }

    // DEPTH
    cv::Mat depthShifted;
    if (status[DEPTH_SD])
    {
        depth.convertTo(images[DEPTH_SD], CV_16U, 1);
        cv::flip(images[DEPTH_SD], images[DEPTH_SD], 1);
    }
    if (status[DEPTH_SD_RECT] || status[DEPTH_QHD] || status[DEPTH_HD])
    {
        depth.convertTo(depthShifted, CV_16U, 1, depthShift);
        cv::flip(depthShifted, depthShifted, 1);
    }
    if (status[DEPTH_SD_RECT])
    {
        cv::remap(depthShifted, images[DEPTH_SD_RECT], map1Ir, map2Ir, cv::INTER_NEAREST);
    }
    if (status[DEPTH_QHD])
    {
        lockRegLowRes.lock();
        depthRegLowRes->registerDepth(depthShifted, images[DEPTH_QHD]);
        lockRegLowRes.unlock();
    }
    if (status[DEPTH_HD])
    {
        lockRegHighRes.lock();
        depthRegHighRes->registerDepth(depthShifted, images[DEPTH_HD]);
        lockRegHighRes.unlock();
    }
}

void Kinect2Bridge::processColor(std::vector<cv::Mat> &images, const std::vector<Status> &status)
{
    // COLOR
    if (status[COLOR_HD_RECT] || status[MONO_HD_RECT])
    {
        try
        {
            cv::remap(images[COLOR_HD], images[COLOR_HD_RECT], map1Color, map2Color, cv::INTER_AREA);
        }
        catch (cv::Exception& ex)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Exception in processColor() while remapping COLOR_HD -> COLOR_HD_RECT: " << ex.what());
        }
    }
    if (status[COLOR_QHD] || status[MONO_QHD])
    {
        try
        {
            cv::resize(images[COLOR_HD], images[COLOR_QHD], sizeLowRes, 0, 0, cv::INTER_AREA);
        }
        catch (cv::Exception& ex)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Exception in processColor() while resizing COLOR_HD -> COLOR_QHD: " << ex.what());
        }
    }
    if (status[COLOR_QHD_RECT] || status[MONO_QHD_RECT])
    {
        try
        {
            cv::remap(images[COLOR_HD], images[COLOR_QHD_RECT], map1LowRes, map2LowRes, cv::INTER_AREA);
        }
        catch (cv::Exception& ex)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Exception in processColor() while remapping COLOR_HD -> COLOR_QHD_RECT: " << ex.what());
        }
    }

    // MONO
    if (status[MONO_HD])
    {
        try
        {
            cv::cvtColor(images[COLOR_HD], images[MONO_HD], CV_BGR2GRAY);
        }
        catch (cv::Exception& ex)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Exception in processColor(): COLOR_HD -> MONO_HD" << ex.what());
        }
    }
    if (status[MONO_HD_RECT])
    {
        try
        {
            cv::cvtColor(images[COLOR_HD_RECT], images[MONO_HD_RECT], CV_BGR2GRAY);
        }
        catch (cv::Exception& ex)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Exception in processColor(): COLOR_HD_RECT -> MONO_HD_RECT" << ex.what());
        }
    }
    if (status[MONO_QHD])
    {
        try
        {
            cv::cvtColor(images[COLOR_QHD], images[MONO_QHD], CV_BGR2GRAY);
        }
        catch (cv::Exception& ex)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Exception in processColor(): COLOR_QHD -> MONO_QHD" << ex.what());
        }
    }
    if (status[MONO_QHD_RECT])
    {
        try
        {
            cv::cvtColor(images[COLOR_QHD_RECT], images[MONO_QHD_RECT], CV_BGR2GRAY);
        }
        catch (cv::Exception& ex)
        {
            ROS_WARN_STREAM_NAMED("kinect2_bridge", "Exception in processColor(): COLOR_QHD_RECT -> MONO_QHD_RECT" << ex.what());
        }
    }
}

void Kinect2Bridge::publishImages(const std::vector<cv::Mat> &images, const std_msgs::Header &header, const std::vector<Status> &status, const size_t frame, size_t &pubFrame, const size_t begin, const size_t end)
{
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "publishImages() - images array size = " << images.size() << "; publishing from index " << begin << " to index " << end);
    std::vector<sensor_msgs::ImagePtr> imageMsgs(COUNT);
    std::vector<sensor_msgs::CompressedImagePtr> compressedMsgs(COUNT);
    sensor_msgs::CameraInfoPtr infoHDMsg,  infoQHDMsg,  infoIRMsg;
    std_msgs::Header _header = header;
    if(begin < COLOR_HD)
    {
        _header.frame_id = baseNameTF + K2_TF_IR_OPT_FRAME;

        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Frame ID for images (IR image): " << baseNameTF + K2_TF_IR_OPT_FRAME);

        infoIRMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
        *infoIRMsg = infoIR;
        infoIRMsg->header = _header;
    }
    else
    {
        _header.frame_id = baseNameTF + K2_TF_RGB_OPT_FRAME;

        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Frame ID for images (color image, 1): " << baseNameTF + K2_TF_RGB_OPT_FRAME);

        infoHDMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
        *infoHDMsg = infoHD;
        infoHDMsg->header = _header;

        infoQHDMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
        *infoQHDMsg = infoQHD;
        infoQHDMsg->header = _header;

    }

    if (end <= images.size())
    {
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Publishing images: " << (end - begin));
        for(size_t i = begin; i < end; ++i)
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "images[" << i << "] size: " << images[i].rows << "x" << images[i].cols);
            if (images[i].rows > 0 && images[i].cols > 0)
            {
                if(i < DEPTH_HD || i == COLOR_SD_RECT)
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Frame ID for images (IR image, 2): " << baseNameTF + K2_TF_IR_OPT_FRAME);
                    _header.frame_id = baseNameTF + K2_TF_IR_OPT_FRAME;
                }
                else
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Frame ID for images (color image, 2): " << baseNameTF + K2_TF_RGB_OPT_FRAME);
                    _header.frame_id = baseNameTF + K2_TF_RGB_OPT_FRAME;
                }

                switch(status[i])
                {
                    case UNSUBCRIBED:
                        break;
                    case RAW:
                        imageMsgs[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);
                        createImage(images[i], _header, Image(i), *imageMsgs[i]);
                        break;
                    case COMPRESSED:
                        compressedMsgs[i] = sensor_msgs::CompressedImagePtr(new sensor_msgs::CompressedImage);
                        createCompressed(images[i], _header, Image(i), *compressedMsgs[i]);
                        break;
                    case BOTH:
                    {
                        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Creating ROS image messages for image " << i);
                        imageMsgs[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);
                        compressedMsgs[i] = sensor_msgs::CompressedImagePtr(new sensor_msgs::CompressedImage);
                        createImage(images[i], _header, Image(i), *imageMsgs[i]);
                        createCompressed(images[i], _header, Image(i), *compressedMsgs[i]);
                        break;
                    }
                }
            }
        }

#ifndef _WIN32
        while(frame != pubFrame)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
#endif

        ROS_INFO_STREAM_NAMED("kinect2_bridge", "Publishing ROS messages...");
        lockPub.lock();
        for(size_t i = begin; i < end; ++i)
        {
            switch(status[i])
            {
                case UNSUBCRIBED:
                    break;
                case RAW:
                    if (imageMsgs[i])
                        imagePubs[(Image) i].publish(imageMsgs[i]);
                    break;
                case COMPRESSED:
                    if (compressedMsgs[i])
                        compressedPubs[(Image) i].publish(compressedMsgs[i]);
                    break;
                case BOTH:
                {
                    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Publishing BOTH image messages for image " << i);
                    if (imageMsgs[i])
                        imagePubs[(Image) i].publish(imageMsgs[i]);

                    if (compressedMsgs[i])
                        compressedPubs[(Image) i].publish(compressedMsgs[i]);

                    break;
                }
            }
        }

        if (begin < COLOR_HD)
        {
            if (infoIRPub.getNumSubscribers() > 0)
            {
                infoIRPub.publish(infoIRMsg);
            }
        }
        else
        {
            if (infoHDPub.getNumSubscribers() > 0)
            {
                infoHDPub.publish(infoHDMsg);
            }
            if (infoQHDPub.getNumSubscribers() > 0)
            {
                infoQHDPub.publish(infoQHDMsg);
            }
        }
        ROS_INFO_STREAM_NAMED("kinect2_bridge", "ROS messages published.");
    }
    else
    {
        ROS_WARN_STREAM_NAMED("kinect2_bridge", "publishImages() got an incorrectly sized images array to publish! Expected size " << end << ", got size " << images.size() << ".");
    }
    ++pubFrame;
    lockPub.unlock();
}

void Kinect2Bridge::createImage(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::Image &msgImage) const
{
    size_t step, size;
    step = image.cols * image.elemSize();
    size = image.rows * step;

    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Creating uncompressed ROS image for image of type " << type << ", size " << image.rows << "x" << image.cols);

    switch (type)
    {
        case IR_SD:
        case IR_SD_RECT:
        case DEPTH_SD:
        case DEPTH_SD_RECT:
        case DEPTH_HD:
        case DEPTH_QHD:
            msgImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            break;
        case COLOR_SD_RECT:
        case COLOR_HD:
        case COLOR_HD_RECT:
        case COLOR_QHD:
        case COLOR_QHD_RECT:
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing COLOR_HD = " << COLOR_HD << ", type = " << type);
            msgImage.encoding = sensor_msgs::image_encodings::BGR8;
            break;
        case MONO_HD:
        case MONO_HD_RECT:
        case MONO_QHD:
        case MONO_QHD_RECT:
            msgImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
            break;
        case COUNT:
            return;
    }

    msgImage.header = header;
    msgImage.height = image.rows;
    msgImage.width = image.cols;
    msgImage.is_bigendian = false;
    msgImage.step = step;
    msgImage.data.resize(size);
    memcpy(msgImage.data.data(), image.data, size);
}

void Kinect2Bridge::createCompressed(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::CompressedImage &msgImage) const
{
    ROS_INFO_STREAM_NAMED("kinect2_bridge", "Creating compressed ROS image for image of type " << type << ", size " << image.rows << "x" << image.cols);
    msgImage.header = header;

    switch (type)
    {
        case IR_SD:
        case IR_SD_RECT:
        case DEPTH_SD:
        case DEPTH_SD_RECT:
        case DEPTH_HD:
        case DEPTH_QHD:
            msgImage.format = compression16BitString;
            cv::imencode(compression16BitExt, image, msgImage.data, compressionParams);
            break;
        case COLOR_SD_RECT:
        case COLOR_HD:
        case COLOR_HD_RECT:
        case COLOR_QHD:
        case COLOR_QHD_RECT:
        {
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Processing COLOR_HD = " << COLOR_HD << ", type = " << type);
            msgImage.format = sensor_msgs::image_encodings::BGR8 + "; jpeg compressed bgr8";
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "Calling cv::imencode...");
            cv::imencode(".jpg", image, msgImage.data, compressionParams);
            ROS_INFO_STREAM_NAMED("kinect2_bridge", "cv::imencode done.");
            break;
        }
        case MONO_HD:
        case MONO_HD_RECT:
        case MONO_QHD:
        case MONO_QHD_RECT:
            msgImage.format = sensor_msgs::image_encodings::TYPE_8UC1 + "; jpeg compressed ";
            cv::imencode(".jpg", image, msgImage.data, compressionParams);
            break;
        case COUNT:
            return;
    }
}

void Kinect2Bridge::publishStaticTF()
{
    setThreadName("TFPublisher");
    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform stColorOpt, stIrOpt;
    tf::StampedTransform stMapToKinect2;
    ros::Time now = ros::Time::now();

    tf::Matrix3x3 rot(m_d->rotation.at<double>(0, 0), m_d->rotation.at<double>(0, 1), m_d->rotation.at<double>(0, 2),
                      m_d->rotation.at<double>(1, 0), m_d->rotation.at<double>(1, 1), m_d->rotation.at<double>(1, 2),
                      m_d->rotation.at<double>(2, 0), m_d->rotation.at<double>(2, 1), m_d->rotation.at<double>(2, 2));

    tf::Quaternion qZero;
    qZero.setRPY(0, 0, 0);
    tf::Vector3 trans(m_d->translation.at<double>(0), m_d->translation.at<double>(1), m_d->translation.at<double>(2));
    tf::Vector3 vZero(0, 0, 0);
    tf::Transform tIr(rot, trans), tZero(qZero, vZero);

    stMapToKinect2 = tf::StampedTransform(tZero, now, std::string("kinect2_origin"), baseNameTF + K2_TF_LINK);

    stColorOpt = tf::StampedTransform(tZero, now, baseNameTF + K2_TF_LINK, baseNameTF + K2_TF_RGB_OPT_FRAME);
    stIrOpt = tf::StampedTransform(tIr, now, baseNameTF + K2_TF_RGB_OPT_FRAME, baseNameTF + K2_TF_IR_OPT_FRAME);

    for(; running && ros::ok();)
    {
        ROS_DEBUG_STREAM_NAMED("kinect2_bridge", "Publishing static TFs.");
        now = ros::Time::now();
        stColorOpt.stamp_ = now;
        stIrOpt.stamp_ = now;

        broadcaster.sendTransform(stMapToKinect2);

        broadcaster.sendTransform(stColorOpt);
        broadcaster.sendTransform(stIrOpt);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

Kinect2BridgeNodelet::Kinect2BridgeNodelet() : Nodelet(), pKinect2Bridge(NULL)
{
}

Kinect2BridgeNodelet::~Kinect2BridgeNodelet()
{
    if(pKinect2Bridge)
    {
        pKinect2Bridge->stop();
        delete pKinect2Bridge;
    }
}

void Kinect2BridgeNodelet::onInit()
{
    // Nodelet won't use image IO mode
    pKinect2Bridge = new Kinect2Bridge(false, getNodeHandle(), getPrivateNodeHandle());
    if(!pKinect2Bridge->start())
    {
        delete pKinect2Bridge;
        pKinect2Bridge = NULL;
        throw nodelet::Exception("Could not start kinect2_bridge!");
    }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Kinect2BridgeNodelet, nodelet::Nodelet)
