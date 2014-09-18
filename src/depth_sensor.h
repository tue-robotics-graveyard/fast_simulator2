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

    void sense(const World& world, const geo::Pose3D& sensor_pose) const;

private:

    int width_, height_;

    geo::DepthCamera camera_;

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
