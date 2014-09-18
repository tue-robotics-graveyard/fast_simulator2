#ifndef ED_SIMULATOR_DEPTH_SENSOR_H_
#define ED_SIMULATOR_DEPTH_SENSOR_H_

#include "object.h"

#include <geolib/sensors/DepthCamera.h>

// ROS
#include <ros/publisher.h>

namespace sim
{

class DepthSensor : public Object
{

public:

    DepthSensor();

    virtual ~DepthSensor();

    void configure(tue::Configuration config);

    void sense(const World& world, const geo::Pose3D& sensor_pose) const;

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

}

#endif
