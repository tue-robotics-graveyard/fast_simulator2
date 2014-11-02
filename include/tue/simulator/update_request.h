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

    void setPose(const LUId& parent, const LUId& child, const geo::Pose3D& pose)
    {
        poses.push_back(boost::make_shared<Transform>(parent, child, pose));
    }

    std::vector<TransformConstPtr> poses;

};

}

#endif
