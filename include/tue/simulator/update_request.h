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

    bool empty() const { return transforms.empty(); }

    LUId addTransform(const LUId& parent, const LUId& child, const geo::Pose3D& pose)
    {
        TransformConstPtr t = boost::make_shared<Transform>(parent, child, pose);
        transforms.push_back(t);
        return t->id();
    }

    void updateTransform(const LUId& transform_id, const geo::Pose3D& pose)
    {
        transform_ups.push_back(std::make_pair<LUId, geo::Pose3D>(transform_id, pose));
    }

    std::vector<TransformConstPtr> transforms;
    std::vector<std::pair<LUId, geo::Pose3D> > transform_ups;

};

}

#endif
