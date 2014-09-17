#ifndef ED_SIMULATOR_OBJECT_H_
#define ED_SIMULATOR_OBJECT_H_

#include "types.h"

#include <geolib/datatypes.h>

namespace sim
{

class Object
{

public:

    Object(const UUID& id);

    /// Returns an updated version of this object. Returns null pointer if object did not change.
    virtual ObjectPtr step(double dt) const;

    const UUID& id() const { return id_; }
    const geo::Pose3D& pose() const { return pose_; }
    geo::ShapeConstPtr shape() const { return shape_; }

private:

    UUID id_;

    std::string type_;

    geo::ShapePtr shape_;

    geo::Pose3D pose_;

    bool moving_;
    geo::Pose3D abs_velocity_;

};

}

#endif
