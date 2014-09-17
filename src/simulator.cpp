#include "simulator.h"

#include "object.h"

namespace sim
{

// ----------------------------------------------------------------------------------------------------

Simulator::Simulator()
{
}

// ----------------------------------------------------------------------------------------------------

Simulator::~Simulator()
{
}

// ----------------------------------------------------------------------------------------------------

void Simulator::step(double dt, std::vector<ObjectConstPtr>& changed_objects)
{
    for(std::map<UUID, ObjectConstPtr>::iterator it = objects_.begin(); it != objects_.end(); ++it)
    {
        const ObjectConstPtr& obj = it->second;

        ObjectConstPtr obj_update = obj->step(dt);
        if (obj_update)
        {
            it->second = obj_update;
            changed_objects.push_back(obj_update);
        }
    }

}

// ----------------------------------------------------------------------------------------------------

void Simulator::addObject(const ObjectConstPtr& object)
{
    objects_[object->id()] = object;
}

}
