#include "simulator.h"

#include "object.h"

#include <ed/models/loader.h>

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

void Simulator::configure(tue::Configuration config)
{
    if (config.readArray("objects"))
    {
        while (config.nextArrayItem())
        {
            std::string id, type;
            if (config.value("id", id) && config.value("type", type))
            {
                geo::Pose3D pose = geo::Pose3D::identity();

                if (config.readGroup("pose"))
                {
                    if (config.value("x", pose.t.x) && config.value("y", pose.t.y) && config.value("z", pose.t.z))
                    {
                        ed::models::Loader l;
                        geo::ShapePtr shape = l.loadShape(type);
                        if (shape)
                        {
                            ObjectPtr obj(new Object(id));
                            obj->setType(type);
                            obj->setPose(pose);
                            obj->setShape(shape);

                            addObject(obj);
                        }
                        else
                        {
                            config.addError("Unknown object type: '" + type + "'.");
                        }
                    }

                    config.endGroup();
                }
                else
                {
                    config.addError("Object does not contain pose");
                }
            }
        }

        config.endArray();
    }
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
