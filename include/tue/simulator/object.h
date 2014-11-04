#ifndef ED_SIMULATOR_OBJECT_H_
#define ED_SIMULATOR_OBJECT_H_

#include "types.h"

#include <geolib/datatypes.h>

#include <tue/config/configuration.h>

namespace sim
{

class Object
{

public:

    Object(const UUId& id = "");

    /// Returns an updated version of this object. Returns null pointer if object did not change.
    virtual ObjectPtr step(const World& world, double dt) const;

    // TODO: move this to seperate Sensor class
    virtual void sense(const World& world, const geo::Pose3D& sensor_pose) const {}

    virtual void configure(tue::Configuration config) {}

    const UUId& id() const { return id_; }
    const std::string& type() const { return type_; }
    geo::ShapeConstPtr shape() const { return shape_; }

    void setShape(const geo::ShapeConstPtr& shape) { shape_ = shape; }
    void setType(const std::string& type) { type_ = type; }

    // TRANSFORMS

    const LUId& parent() const { return parent_; }
    const LUId& parentTransform() const { return parent_transform_; }

    bool getDirectTransform(const LUId& child_id, LUId& transform_id) const;

    void addTransform(const LUId& child_id, const LUId& transform_id) { transforms_[child_id] = transform_id; }
    void setParent(const LUId& parent_id, const LUId& parent_transform_id)
    {
        parent_ = parent_id;
        parent_transform_ = parent_transform_id;
    }
    const std::map<LUId, LUId>& transforms() const { return transforms_; }

private:

    UUId id_;

    std::string type_;

    geo::ShapeConstPtr shape_;

    // TRANSFORMS

    std::map<LUId, LUId> transforms_;
    LUId parent_;
    LUId parent_transform_;

};

}

#endif
