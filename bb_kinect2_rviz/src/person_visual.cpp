/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/Ogre.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include "person_visual.h"

using namespace bb_kinect2_rviz;

PersonVisual::PersonVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Imu's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  parent_node_= parent_node;

  frame_node_->setVisible(false);

  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());

  // Create sphere and cylinder shape placeholders
  createPersonShapes();
}

PersonVisual::~PersonVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void PersonVisual::setMessage(const bb_person_msgs::Persons::ConstPtr& msg)
{
    ROS_INFO_STREAM_NAMED("bb_kinect2_rviz", "New Persons message arrived.");
    /*for (unsigned int k = 0; k < JointType_Count; k++)
    {
        Ogre::Vector3 jointPos(msg->joints3D[k].x, msg->joints3D[k].y, msg->joints3D[k].z);
        joint_spheres_[k]->setPosition(jointPos);

        joint_transforms_[k]->header.stamp = ros::Time::now();
        joint_transforms_[k]->child_frame_id = jointNameFromJointID((JointType) k, msg->tracking_id, msg->tracking_index);

        joint_transforms_[k]->transform.translation.x = msg->joints3D[k].x;
        joint_transforms_[k]->transform.translation.y = msg->joints3D[k].y;
        joint_transforms_[k]->transform.translation.z = msg->joints3D[k].z;

        tf_broadcaster_->sendTransform(*joint_transforms_[k]);
    }

    updateLimbArrow(msg, JointType_SpineBase, JointType_SpineMid, 0);
    updateLimbArrow(msg, JointType_SpineMid, JointType_SpineShoulder, 1);
    updateLimbArrow(msg, JointType_SpineShoulder, JointType_Neck, 2);
    updateLimbArrow(msg, JointType_Neck, JointType_Head, 3);

    updateLimbArrow(msg, JointType_SpineShoulder, JointType_ShoulderLeft, 4);
    updateLimbArrow(msg, JointType_ShoulderLeft, JointType_ElbowLeft, 5);
    updateLimbArrow(msg, JointType_ElbowLeft, JointType_WristLeft, 6);
    updateLimbArrow(msg, JointType_WristLeft, JointType_HandLeft, 7);
    updateLimbArrow(msg, JointType_HandLeft, JointType_HandTipLeft, 8);
    updateLimbArrow(msg, JointType_HandLeft, JointType_ThumbLeft, 9);

    updateLimbArrow(msg, JointType_SpineShoulder, JointType_ShoulderRight, 10);
    updateLimbArrow(msg, JointType_ShoulderRight, JointType_ElbowRight, 11);
    updateLimbArrow(msg, JointType_ElbowRight, JointType_WristRight, 12);
    updateLimbArrow(msg, JointType_WristRight, JointType_HandRight, 13);
    updateLimbArrow(msg, JointType_HandRight, JointType_HandTipRight, 14);
    updateLimbArrow(msg, JointType_HandRight, JointType_ThumbRight, 15);

    updateLimbArrow(msg, JointType_SpineBase, JointType_HipLeft, 16);
    updateLimbArrow(msg, JointType_HipLeft, JointType_KneeLeft, 17);
    updateLimbArrow(msg, JointType_KneeLeft, JointType_AnkleLeft, 18);
    updateLimbArrow(msg, JointType_AnkleLeft, JointType_FootLeft, 19);

    updateLimbArrow(msg, JointType_SpineBase, JointType_HipRight, 20);
    updateLimbArrow(msg, JointType_HipRight, JointType_KneeRight, 21);
    updateLimbArrow(msg, JointType_KneeRight, JointType_AnkleRight, 22);
    updateLimbArrow(msg, JointType_AnkleRight, JointType_FootRight, 23);*/
}

void PersonVisual::updateLimbArrow(const bb_person_msgs::Person::ConstPtr& msg, JointType start, JointType end, unsigned int arrow_index)
{
    if (arrow_index < limb_arrows_.size())
    {
        Ogre::Vector3 point1(msg->joints3D[start].x, msg->joints3D[start].y, msg->joints3D[start].z);
        Ogre::Vector3 point2(msg->joints3D[end].x, msg->joints3D[end].y, msg->joints3D[end].z);

        Ogre::Vector3 direction = point2 - point1;
        float distance = direction.length();

        float head_length_proportion = 0.23; // Seems to be a good value based on default in arrow.h of shaft:head ratio of 1:0.3
        float head_length = head_length_proportion * distance;

        float shaft_length = distance - head_length;

        limb_arrows_[arrow_index]->set(shaft_length, 0.15, head_length, 0.15);

        direction.normalise();

        // for some reason the arrow goes into the y direction by default
        Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(direction);

        limb_arrows_[arrow_index]->setPosition(point1);
        limb_arrows_[arrow_index]->setOrientation(orient);
    }
}

void PersonVisual::setVisible(bool visible)
{
    if (frame_node_)
        frame_node_->setVisible(visible);
}

// Position and orientation are passed through to the SceneNode.
void PersonVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void PersonVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

void PersonVisual::setColor(float r, float g, float b, float a)
{
    for (unsigned int k = 0; k < JointType_Count; k++)
    {
        joint_spheres_[k]->setColor(r, g, b, a);
    }

    for (unsigned int k = 0; k < limb_arrows_.size(); k++)
    {
        limb_arrows_[k]->setColor(r, g, b, a);
    }
}

void PersonVisual::setSphereRadius(float radius)
{
    for (unsigned int k = 0; k < JointType_Count; k++)
    {
        joint_spheres_[k]->setScale(Ogre::Vector3(radius, radius, radius));
    }
}

void PersonVisual::setCylinderRadius(float radius)
{

}

std::string PersonVisual::jointNameFromJointID(JointType type, int tracking_id, int tracking_index)
{
    std::string jointID;
    switch (type)
    {
        case JointType_SpineBase:
            jointID = "SpineBase";
            break;
        case JointType_SpineMid:
            jointID = "SpineMid";
            break;
        case JointType_Neck:
            jointID = "Neck";
            break;
        case JointType_Head:
            jointID = "Head";
            break;
        case JointType_ShoulderLeft:
            jointID = "ShoulderLeft";
            break;
        case JointType_ElbowLeft:
            jointID = "ElbowLeft";
            break;
        case JointType_WristLeft:
            jointID = "WristLeft";
            break;
        case JointType_HandLeft:
            jointID = "HandLeft";
            break;
        case JointType_ShoulderRight:
            jointID = "ShoulderRight";
            break;
        case JointType_ElbowRight:
            jointID = "ElbowRight";
            break;
        case JointType_WristRight:
            jointID = "WristRight";
            break;
        case JointType_HandRight:
            jointID = "HandRight";
            break;
        case JointType_HipLeft:
            jointID = "HipLeft";
            break;
        case JointType_KneeLeft:
            jointID = "KneeLeft";
            break;
        case JointType_AnkleLeft:
            jointID = "AnkleLeft";
            break;
        case JointType_FootLeft:
            jointID = "FootLeft";
            break;
        case JointType_HipRight:
            jointID = "HipRight";
            break;
        case JointType_KneeRight:
            jointID = "KneeRight";
            break;
        case JointType_AnkleRight:
            jointID = "AnkleRight";
            break;
        case JointType_FootRight:
            jointID = "FootRight";
            break;
        case JointType_SpineShoulder:
            jointID = "SpineShoulder";
            break;
        case JointType_HandTipLeft:
            jointID = "HandTipLeft";
            break;
        case JointType_ThumbLeft:
            jointID = "ThumbLeft";
            break;
        case JointType_HandTipRight:
            jointID = "HandTipRight";
            break;
        case JointType_ThumbRight:
            jointID = "ThumbRight";
            break;
        default:
            jointID = "Unknown";
            break;
    }

    if (tracking_id >= 0)
        jointID += "_" + std::to_string(tracking_id);

    if (tracking_index >= 0)
        jointID += "_" + std::to_string(tracking_index);

    return jointID;
}

void PersonVisual::createPersonShapes()
{
    joint_transforms_.resize(JointType_Count);
    for (unsigned int k = 0; k < JointType_Count; k++)
    {
        boost::shared_ptr<rviz::Shape> joint_sphere;
        joint_sphere.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
        joint_sphere->setScale(Ogre::Vector3(0.06, 0.06, 0.06));
        joint_spheres_.push_back(joint_sphere);

        std::string joint_frame_id = jointNameFromJointID((JointType) k);
        joint_transforms_[k].reset(new geometry_msgs::TransformStamped);
        joint_transforms_[k]->header.frame_id = std::string("kinect2_link");
        joint_transforms_[k]->child_frame_id = std::string(joint_frame_id);

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        joint_transforms_[k]->transform.rotation.x = q.x();
        joint_transforms_[k]->transform.rotation.y = q.y();
        joint_transforms_[k]->transform.rotation.z = q.z();
        joint_transforms_[k]->transform.rotation.w = q.w();

        /*std::string labelText = jointNameFromJointID((JointType) k);
        ROS_INFO_STREAM_NAMED("bb_kinect2_rviz", "Adding joint label: " << labelText);

        std::stringstream nodeNameStream;
        nodeNameStream << labelText << "_" << k;
        Ogre::SceneNode* labelNode = frame_node_->createChildSceneNode(Ogre::Vector3(0.04, 0.04, 0.2 + k));
        boost::shared_ptr<rviz::MovableText> joint_label;

        joint_label.reset(new rviz::MovableText(nodeNameStream.str()));
        joint_label->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);
        joint_label->showOnTop(true);
        joint_label->setCharacterHeight(32.0f);
        joint_label->setColor(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
        joint_labels_.push_back(joint_label);

        labelNode->setScale(150.0, 150.0, 150.0);
        labelNode->attachObject(joint_label.get());
        labelNode->setVisible(true);

        joint_label_scene_nodes_.push_back(labelNode);*/

        /*boost::shared_ptr<rviz::Shape> limb_cylinder;
        joint_sphere.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_));
        limb_cylinders_.push_back(limb_cylinder);*/
    }

    // Create arrow shapes for skeleton/limbs
    // JointType_SpineBase, JointType_SpineMid
    boost::shared_ptr<rviz::Arrow> arrow_1;
    arrow_1.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_1);
    // JointType_SpineMid, JointType_SpineShoulder
    boost::shared_ptr<rviz::Arrow> arrow_2;
    arrow_2.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_2);
    // JointType_SpineShoulder, JointType_SpineNeck
    boost::shared_ptr<rviz::Arrow> arrow_3;
    arrow_3.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_3);
    // JointType_SpineNeck, JointType_SpineHead
    boost::shared_ptr<rviz::Arrow> arrow_4;
    arrow_4.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_4);

    // JointType_SpineShoulder, JointType_ShoulderLeft
    boost::shared_ptr<rviz::Arrow> arrow_5;
    arrow_5.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_5);
    // JointType_ShoulderLeft, JointType_ElbowLeft
    boost::shared_ptr<rviz::Arrow> arrow_6;
    arrow_6.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_6);
    // JointType_ElbowLeft, JointType_WristLeft
    boost::shared_ptr<rviz::Arrow> arrow_7;
    arrow_7.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_7);
    // JointType_WristLeft, JointType_HandLeft
    boost::shared_ptr<rviz::Arrow> arrow_8;
    arrow_8.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_8);
    // JointType_HandLeft, JointType_HandTipLeft
    boost::shared_ptr<rviz::Arrow> arrow_9;
    arrow_9.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_9);
    // JointType_HandLeft, JointType_ThumbLeft
    boost::shared_ptr<rviz::Arrow> arrow_10;
    arrow_10.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_10);

    // JointType_SpineShoulder, JointType_ShoulderRight
    boost::shared_ptr<rviz::Arrow> arrow_11;
    arrow_11.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_11);
    // JointType_ShoulderRight, JointType_ElbowRight
    boost::shared_ptr<rviz::Arrow> arrow_12;
    arrow_12.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_12);
    // JointType_ElbowRight, JointType_WristRight
    boost::shared_ptr<rviz::Arrow> arrow_13;
    arrow_13.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_13);
    // JointType_WristRight, JointType_HandRight
    boost::shared_ptr<rviz::Arrow> arrow_14;
    arrow_14.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_14);
    // JointType_HandRight, JointType_HandTipRight
    boost::shared_ptr<rviz::Arrow> arrow_15;
    arrow_15.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_15);
    // JointType_HandRight, JointType_ThumbRight
    boost::shared_ptr<rviz::Arrow> arrow_16;
    arrow_16.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_16);

    // JointType_SpineBase, JointType_HipLeft
    boost::shared_ptr<rviz::Arrow> arrow_17;
    arrow_17.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_17);
    // JointType_HipLeft, JointType_KneeLeft
    boost::shared_ptr<rviz::Arrow> arrow_18;
    arrow_18.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_18);
    // JointType_KneeLeft, JointType_AnkleLeft
    boost::shared_ptr<rviz::Arrow> arrow_19;
    arrow_19.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_19);
    // JointType_AnkleLeft, JointType_FootLeft
    boost::shared_ptr<rviz::Arrow> arrow_20;
    arrow_20.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_20);

    // JointType_SpineBase, JointType_HipRight
    boost::shared_ptr<rviz::Arrow> arrow_21;
    arrow_21.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_21);
    // JointType_HipRight, JointType_KneeRight
    boost::shared_ptr<rviz::Arrow> arrow_22;
    arrow_22.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_22);
    // JointType_KneeRight, JointType_AnkleRight
    boost::shared_ptr<rviz::Arrow> arrow_23;
    arrow_23.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_23);
    // JointType_AnkleRight, JointType_FootRight
    boost::shared_ptr<rviz::Arrow> arrow_24;
    arrow_24.reset(new rviz::Arrow(scene_manager_, frame_node_));
    limb_arrows_.push_back(arrow_24);
}
