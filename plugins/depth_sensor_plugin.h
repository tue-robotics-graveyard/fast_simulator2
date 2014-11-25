#ifndef SIMULATOR_DEPTH_SENSOR_PLUGIN_H_
#define SIMULATOR_DEPTH_SENSOR_PLUGIN_H_

#include "fast_simulator2/plugin.h"

#include <geolib/sensors/DepthCamera.h>

// ROS
#include <ros/publisher.h>

class DepthSensorPlugin : public sim::Plugin
{

public:

    DepthSensorPlugin();

    void configure(tue::Configuration config, const sim::LUId& obj_id);

    void process(const sim::World& world, const sim::LUId& obj_id, double dt, sim::UpdateRequest& req);

private:

    bool render_rgb_, render_depth_;

    int rgb_width_, rgb_height_;
    int depth_width_, depth_height_;

    geo::DepthCamera depth_rasterizer_;

    // ROS
    std::vector<ros::Publisher> pubs_rgb_;
    std::vector<ros::Publisher> pubs_depth_;
    std::vector<ros::Publisher> pubs_cam_info_rgb_;
    std::vector<ros::Publisher> pubs_cam_info_depth_;
    std::vector<ros::Publisher> pubs_rgbd_;

    std::string rgb_frame_id_, depth_frame_id_;

};

#endif
