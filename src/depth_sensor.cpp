#include "depth_sensor.h"

namespace sim
{

// ----------------------------------------------------------------------------------------------------

DepthSensor::DepthSensor()
{
}

// ----------------------------------------------------------------------------------------------------

DepthSensor::~DepthSensor()
{
}

// ----------------------------------------------------------------------------------------------------

void DepthSensor::sense(const World& world, const geo::Pose3D& sensor_pose) const
{
    std::cout << "Depth Sensor: " << sensor_pose << std::endl;
}

}
