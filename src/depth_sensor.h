#ifndef ED_SIMULATOR_DEPTH_SENSOR_H_
#define ED_SIMULATOR_DEPTH_SENSOR_H_

#include "object.h"

#include <geolib/sensors/DepthCamera.h>

// ROS
#include <sensor_msgs/CameraInfo.h>

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
    sensor_msgs::CameraInfo cam_info_depth_;
    sensor_msgs::CameraInfo cam_info_rgb_;

};

}

#endif
