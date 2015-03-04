#include "base_controller.h"

#include <ed/world_model.h>
#include <ed/uuid.h>

#include <ros/time.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/ros/msg_conversions.h>

#include <ed/update_request.h>
#include <ed/relation.h>

// ----------------------------------------------------------------------------------------------------

class TransformRelation : public ed::Relation
{

public:

    TransformRelation(const geo::Pose3D& t) : t_(t) {}

    ed::Time latestTime() const { return ed::Time(0); }  // TODO

    bool calculateTransform(const ed::Time& t, geo::Pose3D& tf) const
    {
        tf = t_;
        return true;
    }

private:

    geo::Pose3D t_;

};

// ----------------------------------------------------------------------------------------------------

BaseController::BaseController() : tf_broadcaster_(0)
{
    vel_trans_ = geo::Vec2(0, 0);
    vel_angular_ = 0;
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

    // Init ROS (if needed)
    if (!ros::isInitialized())
         ros::init(ros::M_string(), "simulator", ros::init_options::NoSigintHandler);

    std::string ref_topic;
    if (config.value("base_reference_topic", ref_topic))
    {
        ros::NodeHandle nh;

        ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<geometry_msgs::Twist>
                (ref_topic, 10, boost::bind(&BaseController::referenceCallback, this, _1), ros::VoidPtr(), &cb_queue_);

        sub_ref_ = nh.subscribe(sub_options);
    }
}

// ----------------------------------------------------------------------------------------------------

void BaseController::process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req)
{
    // Get ROS current time
    ros::Time time = ros::Time::now();

    geo::Pose3D base_pose;
    if (!world.calculateTransform("world", ed::UUID(obj_id.id), time.toSec(), base_pose))
    {
        std::cout << "[FAST SIMULATOR 2] Could not get robot base pose" << std::endl;
        return;
    }

    cb_queue_.callAvailable();

    geo::Transform delta;
    delta.t = geo::Vector3(dt * vel_trans_.x, dt * vel_trans_.y, 0);
    delta.R.setRPY(0, 0, dt * vel_angular_);

    base_pose = base_pose * delta;

    tf::StampedTransform tf_odom;
    tf_odom.frame_id_ = "/amigo/odom";
    tf_odom.child_frame_id_ = "/amigo/base_link";
    tf_odom.stamp_ = time;

    geo::convert(base_pose, tf_odom);
    tf_broadcaster_->sendTransform(tf_odom);

    std::cout << base_pose << std::endl;

    // Set transformation
    boost::shared_ptr<TransformRelation> r(new TransformRelation(base_pose));
    req.setRelation("world", ed::UUID(obj_id.id), r);
}

// ----------------------------------------------------------------------------------------------------

void BaseController::referenceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    vel_trans_.x = msg->linear.x;
    vel_trans_.y = msg->linear.y;
    vel_angular_ = msg->angular.z;

    std::cout << *msg << std::endl;
}

SIM_REGISTER_PLUGIN(BaseController)
