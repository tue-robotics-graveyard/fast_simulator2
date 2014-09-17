#ifndef ED_SIMULATOR_DEPTH_SENSOR_H_
#define ED_SIMULATOR_DEPTH_SENSOR_H_

#include "object.h"

#include <geolib/sensors/DepthCamera.h>

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

};

}

#endif
