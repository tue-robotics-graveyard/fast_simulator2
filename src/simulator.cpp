#include "simulator.h"

#include "world.h"
#include "object.h"
#include "robot.h"

#include <ed/models/models.h>

namespace sim
{

// ----------------------------------------------------------------------------------------------------

Simulator::Simulator() : world_(new World())
{
}

// ----------------------------------------------------------------------------------------------------

Simulator::~Simulator()
{
}

// ----------------------------------------------------------------------------------------------------

void Simulator::configure(tue::Configuration config)
{
    if (config.readGroup("robot"))
    {
        RobotPtr robot(new Robot());
        robot->configure(config.limitScope());

        if (!config.hasError())
        {
            robot_ = robot;
        }

        config.endGroup();
    }

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
                        ed::models::NewEntityConstPtr e_created = ed::models::create(type, tue::Configuration(), id);
                        if (e_created && e_created->shape)
                        {
                            ObjectPtr obj(new Object(id));
                            obj->setType(type);
                            obj->setPose(pose);
                            obj->setShape(e_created->shape);

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
    WorldPtr world_updated(new World(*world_));

    for(std::map<UUID, ObjectConstPtr>::const_iterator it = world_->objects.begin(); it != world_->objects.end(); ++it)
    {
        const ObjectConstPtr& obj = it->second;

        ObjectConstPtr obj_update = obj->step(*world_, dt);
        if (obj_update)
        {
            world_updated->objects[it->first] = obj_update;
            changed_objects.push_back(obj_update);
        }
    }

    if (robot_)
    {
        // Update the robot sensors
        std::vector<ObjectConstPtr> sensors;
        std::vector<geo::Pose3D> sensor_poses;

        robot_->getSensors(sensors, sensor_poses);

        for(unsigned int i = 0; i < sensors.size(); ++i)
        {
            sensors[i]->sense(*world_, sensor_poses[i]);
        }
    }


    world_ = world_updated;
}

// ----------------------------------------------------------------------------------------------------

void Simulator::addObject(const ObjectConstPtr& object)
{
    world_->objects[object->id()] = object;
}

}
