#ifndef ED_SIMULATOR_UPDATE_REQUEST_H_
#define ED_SIMULATOR_UPDATE_REQUEST_H_

#include "types.h"

#include <geolib/datatypes.h>

namespace sim
{

class UpdateRequest
{

public:

    UpdateRequest() {}

    bool empty() const { return poses.empty(); }

    void setPose(const UUID& id, const geo::Pose3D& pose)
    {
        poses[id] = pose;
    }

    std::map<UUID, geo::Pose3D> poses;

};

}

#endif
