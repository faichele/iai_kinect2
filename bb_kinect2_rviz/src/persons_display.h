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

#ifndef PERSONS_DISPLAY_H
#define PERSONS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <ros/ros.h>
#include <bb_person_msgs/Person.h>
#include <bb_person_msgs/Persons.h>
#include <bb_person_msgs/TrackingStates.h>
#endif

namespace Ogre
{
    class SceneNode;
}

namespace rviz
{
    class ColorProperty;
    class FloatProperty;
    class IntProperty;
}

namespace bb_kinect2_rviz
{
    class PersonVisual;
    class PersonsDisplay: public rviz::MessageFilterDisplay<bb_person_msgs::Persons>
    {
        Q_OBJECT
        public:
            // Constructor.  pluginlib::ClassLoader creates instances by calling
            // the default constructor, so make sure you have one.
            PersonsDisplay();
            virtual ~PersonsDisplay();

            // Overrides of protected virtual functions from Display.  As much
            // as possible, when Displays are not enabled, they should not be
            // subscribed to incoming data and should not show anything in the
            // 3D view.  These functions are where these connections are made
            // and broken.
        protected:
            virtual void onInitialize();

            // A helper to clear this display back to the initial state.
            virtual void reset();

            // These Qt slots get connected to signals indicating changes in the user-editable properties.
        private Q_SLOTS:
            void updateColorAndAlpha();
            void updateSphereRadius();
            void updateCylinderRadius();

            // Function to handle an incoming ROS message.
        private:
            void processMessage(const bb_person_msgs::Persons::ConstPtr& msg);
            void processTrackingStates(const bb_person_msgs::TrackingStates::ConstPtr& msg);

            // Storage for the list of Person visuals.
            std::vector<boost::shared_ptr<PersonVisual>> visuals_;

            static constexpr unsigned int num_max_persons_ = 6;

            ros::NodeHandle node_handle_;
            ros::Subscriber tracking_states_sub_;

            // User-editable property variables.
            rviz::ColorProperty* color_property_;
            rviz::FloatProperty* alpha_property_;
            rviz::FloatProperty* sphere_radius_property_;
            rviz::FloatProperty* cylinder_radius_property_;
    };
}

#endif // PERSONS_DISPLAY_H
