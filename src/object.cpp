#include "tue/simulator/object.h"

namespace sim
{

// ----------------------------------------------------------------------------------------------------

Object::Object(const UUId& id)
    : id_(id)
{
}

// ----------------------------------------------------------------------------------------------------

ObjectPtr Object::step(const World& world, double dt) const
{
//    if (moving_)
//    {
//        // Create a copy of current object
//        ObjectPtr obj(new Object(*this));

//        // Update position
//        obj->pose_.t += dt * abs_velocity_.t;

//        // TODO: update orientation
//        // ...

//        return obj;
//    }

    return ObjectPtr();
}

// ----------------------------------------------------------------------------------------------------

bool Object::getDirectTransform(const LUId& child_id, LUId& transform_id) const
{
    std::map<LUId, LUId>::const_iterator it = transforms_.find(child_id);
    if (it == transforms_.end())
        return false;

    transform_id = it->second;
    return true;
}

}
