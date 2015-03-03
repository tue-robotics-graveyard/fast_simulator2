#include "base_controller.h"

#include <ed/world_model.h>
#include <ed/uuid.h>

#include <ros/time.h>

#include <geolib/ros/tf_conversions.h>

// ----------------------------------------------------------------------------------------------------

BaseController::BaseController() : tf_broadcaster_(0)
{
}

// ----------------------------------------------------------------------------------------------------

BaseController::~BaseController()
{
    delete tf_broadcaster_;
}


// ----------------------------------------------------------------------------------------------------

void BaseController::configure(tue::Configuration config, const sim::LUId& obj_id)
{
    delete tf_broadcaster_;
    tf_broadcaster_ = new tf::TransformBroadcaster;
}

// ----------------------------------------------------------------------------------------------------

void BaseController::process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req)
{
    // Get ROS current time
    ros::Time time = ros::Time::now();

    geo::Pose3D base_pose;
    if (world.calculateTransform("world", ed::UUID(obj_id.id), time.toSec(), base_pose))
    {
        std::cout << base_pose << std::endl;

        tf::StampedTransform tf_odom;
        tf_odom.frame_id_ = "/amigo/odom";
        tf_odom.child_frame_id_ = "/amigo/base_link";
        tf_odom.stamp_ = time;

        geo::convert(base_pose, tf_odom);
        tf_broadcaster_->sendTransform(tf_odom);
    }

//    req.addTransform(world.rootId(), obj_id, geo::Pose3D(2, 0, 0));
}

SIM_REGISTER_PLUGIN(BaseController)
