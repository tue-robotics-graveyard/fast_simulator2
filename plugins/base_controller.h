#ifndef SIMULATOR_BASE_CONTROLLER_H_
#define SIMULATOR_BASE_CONTROLLER_H_

#include "fast_simulator2/plugin.h"

#include <tf/transform_broadcaster.h>

#include <ros/callback_queue.h>

class BaseController : public sim::Plugin
{

public:

    BaseController();

    ~BaseController();

    void configure(tue::Configuration config, const sim::LUId& obj_id);

    void process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req);

private:

    tf::TransformBroadcaster* tf_broadcaster_;

    ros::Subscriber sub_ref_;

    ros::CallbackQueue cb_queue_;

    void referenceCallback(const geometry_msgs::Twist::ConstPtr& msg);

    geo::Vec2 vel_trans_;
    double vel_angular_;

};

#endif
