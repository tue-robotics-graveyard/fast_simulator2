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
        poses.push_back(std::pair<LUId, Joint>(parent, Joint(child, pose)));
    }

    std::vector<std::pair<LUId, Joint> > poses;

};

}

#endif
