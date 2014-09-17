#include "object.h"

namespace sim
{

// ----------------------------------------------------------------------------------------------------

Object::Object(const UUID& id)
    : id_(id), moving_(false), abs_velocity_(geo::Pose3D::identity())
{

}

// ----------------------------------------------------------------------------------------------------

ObjectPtr Object::step(const World& world, double dt) const
{
    if (moving_)
    {
        // Create a copy of current object
        ObjectPtr obj(new Object(*this));

        // Update position
        obj->pose_.t += dt * abs_velocity_.t;

        // TODO: update orientation
        // ...

        return obj;
    }

    return ObjectPtr();
}

}
