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

#ifndef PERSON_VISUAL_H
#define PERSON_VISUAL_H

#include <bb_person_msgs/Person.h>
#include <bb_person_msgs/Persons.h>
#include <Ogre/Ogre.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

namespace rviz
{
    class Arrow;
    class Shape;
    class MovableText;
}

namespace bb_kinect2_rviz
{
    /* Copied from Kinect2 SDK as to not introduce a dependency to it here. */
    enum JointType
    {
        JointType_SpineBase	= 0,
        JointType_SpineMid	= 1,
        JointType_Neck	= 2,
        JointType_Head	= 3,
        JointType_ShoulderLeft	= 4,
        JointType_ElbowLeft	= 5,
        JointType_WristLeft	= 6,
        JointType_HandLeft	= 7,
        JointType_ShoulderRight	= 8,
        JointType_ElbowRight	= 9,
        JointType_WristRight	= 10,
        JointType_HandRight	= 11,
        JointType_HipLeft	= 12,
        JointType_KneeLeft	= 13,
        JointType_AnkleLeft	= 14,
        JointType_FootLeft	= 15,
        JointType_HipRight	= 16,
        JointType_KneeRight	= 17,
        JointType_AnkleRight	= 18,
        JointType_FootRight	= 19,
        JointType_SpineShoulder	= 20,
        JointType_HandTipLeft	= 21,
        JointType_ThumbLeft	= 22,
        JointType_HandTipRight	= 23,
        JointType_ThumbRight	= 24,
        JointType_Count	= ( JointType_ThumbRight + 1 )
    };

    class PersonVisual
    {
        public:
            // Constructor.  Creates the visual stuff and puts it into the
            // scene, but in an unconfigured state.
            PersonVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

            // Destructor.  Removes the visual stuff from the scene.
            virtual ~PersonVisual();

            // Configure the visual to show the data in the message.
            void setMessage(const bb_person_msgs::Persons::ConstPtr& msg);

            // Set the pose of the coordinate frame the message refers to.
            void setFramePosition(const Ogre::Vector3& position);
            void setFrameOrientation(const Ogre::Quaternion& orientation);

            // Set the color and alpha of the visual.
            void setColor(float r, float g, float b, float a);

            void setSphereRadius(float radius);
            void setCylinderRadius(float radius);

            void setVisible(bool visible);

        private:
            void createPersonShapes();

            std::string jointNameFromJointID(JointType type, int tracking_id = -1, int tracking_index = -1);
            void updateLimbArrow(const bb_person_msgs::Person::ConstPtr& msg, JointType start, JointType end, unsigned int arrow_index);

            // The objects implementing the actual "stick figure" representing a person
            std::vector<boost::shared_ptr<rviz::Shape>> joint_spheres_;
            std::vector<boost::shared_ptr<rviz::Arrow>> limb_arrows_;

            boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            std::vector<boost::shared_ptr<geometry_msgs::TransformStamped>> joint_transforms_;

            // A SceneNode whose pose is set to match the coordinate frame of a person
            Ogre::SceneNode* frame_node_;
            Ogre::SceneNode* parent_node_;

            // The SceneManager, kept here only so the destructor can ask it to
            // destroy the ``frame_node_``.
            Ogre::SceneManager* scene_manager_;
};

}

#endif // PERSON_VISUAL_H
