#include "simulator.h"

#include "tue/simulator/world.h"
#include "tue/simulator/object.h"
#include "robot.h"

// Plugin loading
#include "tue/simulator/plugin.h"
#include "plugin_container.h"
#include <tue/filesystem/path.h>

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
    // Stop all plugins
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        it->second.reset();
    }
}

// ----------------------------------------------------------------------------------------------------

void addObjectRecursive(Simulator& sim, const ed::models::NewEntityConstPtr& e, const geo::Pose3D& pose)
{
    if (e->shape)
    {
        ObjectPtr obj(new Object(e->id));
        obj->setType(e->id);
        obj->setPose(pose * e->pose);
        obj->setShape(e->shape);
        sim.addObject(obj);
    }

    for(std::vector<ed::models::NewEntityPtr>::const_iterator it = e->children.begin(); it != e->children.end(); ++it)
    {
        addObjectRecursive(sim, *it, pose * e->pose);
    }
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

//                        std::cout << "Loading" << std::endl;
                        ed::models::NewEntityConstPtr e_created = ed::models::create(type, tue::Configuration(), id);
//                        std::cout << "Done" << std::endl;
                        if (e_created)
                        {
                            addObjectRecursive(*this, e_created, pose);
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

                if (config.readArray("plugins"))
                {
                    while (config.nextArrayItem())
                    {
                        std::string name, lib_filename;
                        if (config.value("name", name) & config.value("lib", lib_filename))
                        {
                            tue::Configuration plugin_cfg;
                            plugin_cfg.setValue("_object", id);

                            if (config.readGroup("parameters"))
                            {
                                plugin_cfg.add(config);
                                config.endGroup();
                            }

                            // Load with no parameters
                            std::string load_error;
                            loadPlugin(name, lib_filename, plugin_cfg, load_error);

                            if (!load_error.empty())
                            {
                                config.addError(load_error);
                            }
                        }
                    }
                    config.endArray();
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

    // - - - - - - - - - - - - - - -

    // collect all update requests
    std::vector<PluginContainerPtr> plugins_with_requests;
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        const PluginContainerPtr& c = it->second;

        if (c->updateRequest())
        {
//            update(*c->updateRequest());
            plugins_with_requests.push_back(c);
        }
    }

    // Set the new (updated) world
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        const PluginContainerPtr& c = it->second;
        c->setWorld(world_);
    }

    // Clear the requests of all plugins that had requests (which flags them to continue processing)
    for(std::vector<PluginContainerPtr>::iterator it = plugins_with_requests.begin(); it != plugins_with_requests.end(); ++it)
    {
        PluginContainerPtr c = *it;
        c->clearUpdateRequest();
    }
}

// ----------------------------------------------------------------------------------------------------

void Simulator::addObject(const ObjectConstPtr& object)
{
    world_->objects[object->id()] = object;
}

// ----------------------------------------------------------------------------------------------------

PluginContainerPtr Simulator::loadPlugin(const std::string plugin_name, const std::string& lib_filename,
                                         tue::Configuration config, std::string& error)
{
    if (lib_filename.empty())
    {
        error += "Empty library file given.";
        return PluginContainerPtr();
    }

    std::string full_lib_file = lib_filename;
    //        if (lib_filename[0] != '/')
    //        {
    //            // library file is relative
    //            full_lib_file = getFullLibraryPath(lib_filename);
    //            if (full_lib_file.empty())
    //            {
    //                error += "Could not find plugin '" + lib_filename + "'.";
    //                return PluginContainerPtr();
    //            }
    //        }

    if (!tue::filesystem::Path(full_lib_file).exists())
    {
        error += "Could not find plugin '" + full_lib_file + "'.";
        return PluginContainerPtr();
    }

    PluginContainerPtr container(new PluginContainer());
    if (container->loadPlugin(plugin_name, lib_filename, config, error))
    {
        plugin_containers_[plugin_name] = container;
        container->runThreaded();
        return container;
    }

    return PluginContainerPtr();
}


}
