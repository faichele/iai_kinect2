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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "person_visual.h"
#include "persons_display.h"

using namespace bb_kinect2_rviz;

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
PersonsDisplay::PersonsDisplay()
{
    color_property_ = new rviz::ColorProperty( "Color", QColor(204, 51, 204),
                                               "Color to draw person visuals in.",
                                               this, SLOT(updateColorAndAlpha()));

    alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                               "0 is fully transparent, 1.0 is fully opaque.",
                                               this, SLOT(updateColorAndAlpha()));

    sphere_radius_property_ = new rviz::FloatProperty( "Sphere radius", 0.06,
                                               "Radius of spheres visualizing joint keypoints.",
                                               this, SLOT(updateSphereRadius()));

    cylinder_radius_property_ = new rviz::FloatProperty( "Cylinder radius", 0.03,
                                               "Radius of cylinders visualizing joint keypoint connections.",
                                               this, SLOT(updateCylinderRadius()));

    node_handle_ = ros::NodeHandle();
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void PersonsDisplay::onInitialize()
{
    ROS_INFO_STREAM_NAMED("bb_kinect2_rviz", "PersonsDisplay onInitialize()");
    MFDClass::onInitialize();

    // Pre-allocate Person visuals up to the max. number of persons a Kinect2 can track
    for (unsigned int k = 0; k < num_max_persons_; k++)
    {
        boost::shared_ptr<bb_kinect2_rviz::PersonVisual> person_visual;
        person_visual.reset(new PersonVisual(context_->getSceneManager(), scene_node_));
     
        person_visual->setTrackingID(k);
        visuals_.push_back(person_visual);
    }
}

PersonsDisplay::~PersonsDisplay()
{
    visuals_.clear();
}

// Clear the visuals by deleting their objects.
void PersonsDisplay::reset()
{
    MFDClass::reset();
    visuals_.clear();

    for (unsigned int k = 0; k < num_max_persons_; k++)
    {
        boost::shared_ptr<bb_kinect2_rviz::PersonVisual> person_visual;
        person_visual.reset(new PersonVisual(context_->getSceneManager(), scene_node_));
        visuals_.push_back(person_visual);
    }

    tracking_states_sub_.shutdown();
    tracking_states_sub_ = node_handle_.subscribe("/kinect2/tracking_states", 5, &PersonsDisplay::processTrackingStates, this);
}

// Set the current color and alpha values for each visual.
void PersonsDisplay::updateColorAndAlpha()
{
    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();

    for (size_t i = 0; i < visuals_.size(); i++)
    {
        visuals_[i]->setColor(color.r, color.g, color.b, alpha);
    }
}

void PersonsDisplay::updateSphereRadius()
{
    float sphere_radius = sphere_radius_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
    {
        visuals_[i]->setSphereRadius(sphere_radius);
    }
}

void PersonsDisplay::updateCylinderRadius()
{
    float cylinder_radius = cylinder_radius_property_->getFloat();
    for (size_t i = 0; i < visuals_.size(); i++)
    {
        visuals_[i]->setCylinderRadius(cylinder_radius);
    }
}

void PersonsDisplay::processTrackingStates(const bb_person_msgs::TrackingStates::ConstPtr& msg)
{
    ROS_INFO_STREAM_NAMED("bb_kinect2_rviz", "Received new TrackingStates message.");
    for (size_t k = 0; k < msg->tracking_states.size(); k++)
    {
        if (k < visuals_.size())
        {
            visuals_[k]->setVisible(msg->tracking_states[k]);
        }
    }

}

// This is our callback to handle an incoming message.
void PersonsDisplay::processMessage(const bb_person_msgs::Persons::ConstPtr& msg)
{
    ROS_INFO_STREAM_NAMED("bb_kinect2_rviz", "Received new Persons message.");
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation))
    {
        ROS_WARN_STREAM_NAMED("bb_kinect2_rviz", "Error transforming from frame '" << msg->header.frame_id << "' to frame '" << qPrintable(fixed_frame_) << "'");
        return;
    }

    // We are keeping a circular buffer of visual pointers.  This gets
    // the next one, or creates and stores it if the buffer is not full
    for (unsigned int k = 0; k < msg->persons.size(); k++)
    {
        if (k < visuals_.size())
        {
            boost::shared_ptr<PersonVisual> visual = visuals_.at(k);

            // Now set or update the contents of the chosen visual.
            visual->setMessage(msg->persons[k]);
            visual->setFramePosition(position);
            visual->setFrameOrientation(orientation);

            float alpha = alpha_property_->getFloat();
            Ogre::ColourValue color = color_property_->getOgreColor();
            visual->setColor(color.r, color.g, color.b, alpha);
        }
    }
    
}

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bb_kinect2_rviz::PersonsDisplay, rviz::Display)
