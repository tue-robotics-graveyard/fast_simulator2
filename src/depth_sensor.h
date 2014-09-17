#ifndef ED_SIMULATOR_DEPTH_SENSOR_H_
#define ED_SIMULATOR_DEPTH_SENSOR_H_

#include "object.h"

namespace sim
{

class DepthSensor : public Object
{

public:

    DepthSensor();

    virtual ~DepthSensor();

    void sense(const World& world, const geo::Pose3D& sensor_pose) const;

};

}

#endif
