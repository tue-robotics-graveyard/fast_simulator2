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
    const geo::Pose3D& pose() const { return pose_; }
    geo::ShapeConstPtr shape() const { return shape_; }

    void setPose(const geo::Pose3D& pose) { pose_ = pose; }
    void setShape(const geo::ShapeConstPtr& shape) { shape_ = shape; }
    void setType(const std::string& type) { type_ = type; }

private:

    UUId id_;

    std::string type_;

    geo::ShapeConstPtr shape_;

    geo::Pose3D pose_;

    bool moving_;
    geo::Pose3D abs_velocity_;

    std::map<UUId, Joint> joints_;

};

}

#endif
