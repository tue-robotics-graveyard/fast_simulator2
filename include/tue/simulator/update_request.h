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

    inline bool empty() const { return objects.empty() && transforms.empty() && transform_ups.empty(); }

    LUId addObject(const ObjectConstPtr& obj);

    void updateObject(const LUId& id, const ObjectConstPtr& obj)
    {
        objects.push_back(std::pair<LUId, ObjectConstPtr>(id, obj));
    }

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

    std::vector<std::pair<LUId, ObjectConstPtr> > objects;
    std::vector<TransformConstPtr> transforms;
    std::vector<std::pair<LUId, geo::Pose3D> > transform_ups;

};

}

#endif
