#ifndef FAST_SIMULATOR2_LASER_RANGE_FINDER_H_
#define FAST_SIMULATOR2_LASER_RANGE_FINDER_H_

#include "fast_simulator2/object.h"

#include <geolib/sensors/LaserRangeFinder.h>

namespace sim
{

class LaserRangeFinder : public Object
{

public:

    LaserRangeFinder();

    virtual ~LaserRangeFinder();

    void configure(tue::Configuration config);

    void sense(const World& world, const geo::Pose3D& sensor_pose) const;

private:

    geo::LaserRangeFinder lrf_;

};

}

#endif
