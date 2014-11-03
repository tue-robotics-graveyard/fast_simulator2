#include "simulator.h"

#include "tue/simulator/world.h"
#include "tue/simulator/object.h"
#include "tue/simulator/update_request.h"
#include "robot.h"

// Plugin loading
#include "tue/simulator/plugin.h"
#include "plugin_container.h"
#include <tue/filesystem/path.h>

// Object creation
#include <tue/config/loaders/yaml.h>

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

void addObjectRecursive(UpdateRequest& req, const ed::models::NewEntityConstPtr& e, const geo::Pose3D& pose)
{
    if (e->shape)
    {
        ObjectPtr obj(new Object(e->id));
        obj->setType(e->id);
        obj->setShape(e->shape);

        req.addObject(obj);
        req.addTransform(LUId("world"), obj->id(), pose * e->pose);
    }

    for(std::vector<ed::models::NewEntityPtr>::const_iterator it = e->children.begin(); it != e->children.end(); ++it)
    {
        addObjectRecursive(req, *it, pose * e->pose);
    }
}

// ----------------------------------------------------------------------------------------------------


tue::Configuration expandObjectConfig(const std::map<std::string, std::string>& models, tue::Configuration config)
{
    std::string type;
    if (config.value("type", type, tue::OPTIONAL))
    {
        std::map<std::string, std::string>::const_iterator it = models.find(type);
        if (it != models.end())
        {
            tue::Configuration cfg;
            tue::config::loadFromYAMLString(it->second, cfg);
            cfg.add(config);

            return cfg;
        }
        else
        {
            config.addError("Unknown object type: '" + type + "'.");
        }
    }
    else
    {
        return config;
    }
}

// ----------------------------------------------------------------------------------------------------

void createObject(UpdateRequest& req, const std::map<std::string, std::string>& models,
                  const std::string& id, const std::string& type,
                  const geo::Pose3D& pose, tue::Configuration config)
{
    // Try own models first
    std::map<std::string, std::string>::const_iterator it = models.find(type);
    if (it != models.end())
    {
        tue::Configuration cfg = expandObjectConfig(models, config);
        std::cout << cfg << std::endl;

        //  Composition
        if (cfg.readArray("children"))
        {
            while(cfg.nextArrayItem())
            {
                std::string child_id, child_type;
                if (config.value("id", child_id) && config.value("type", child_type))
                {
                    createObject(req, models, child_id, child_type, geo::Pose3D(), cfg);
                }
            }
            cfg.endArray();
        }

//        ObjectPtr e;
//        std::string type;
//        if (cfg.value("type", type, tue::OPTIONAL))
//        {
//            e = createObject(sim, models, id, type, pose, config);
//        }
//        else
//        {
//            e = ObjectPtr(new Object(id));
//            e->setType(type);
//            e->setPose(pose);
//            sim.addObject(e);
//        }

//        return e;
    }
    else // Try the ED database
    {
        ed::models::NewEntityConstPtr e = ed::models::create(type, config, id);
        if (e)
        {
            addObjectRecursive(req, e, pose);
        }
        else
        {
            config.addError("Unknown object type: '" + type + "'.");
        }
    }

//    return ObjectPtr();
}

// ----------------------------------------------------------------------------------------------------

void Simulator::configure(tue::Configuration config)
{
    UpdateRequest req;

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

    if (config.readArray("models"))
    {
        while (config.nextArrayItem())
        {
            std::string name;
            if (config.value("name", name))
            {
                models_[name] = config.toYAMLString();
            }
        }
        config.endArray();
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
                    config.value("x", pose.t.x);
                    config.value("y", pose.t.y);
                    config.value("z", pose.t.z);
                    config.endGroup();
                }
                else
                {
                    config.addError("Object does not contain pose");
                }

                if (!config.hasError())
                    createObject(req, models_, id, type, pose, config);

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

    if (!req.empty())
    {
        WorldPtr world_updated = boost::make_shared<World>(*world_);   // Create a world copy
        world_updated->update(req);
        world_ = world_updated;
    }

}

// ----------------------------------------------------------------------------------------------------

void Simulator::step(double dt, std::vector<ObjectConstPtr>& changed_objects)
{
//    WorldPtr world_updated(new World(*world_));

//    for(std::map<UUID, ObjectConstPtr>::const_iterator it = world_->objects.begin(); it != world_->objects.end(); ++it)
//    {
//        const ObjectConstPtr& obj = it->second;

//        ObjectConstPtr obj_update = obj->step(*world_, dt);
//        if (obj_update)
//        {
//            world_updated->objects[it->first] = obj_update;
//            changed_objects.push_back(obj_update);
//        }
//    }

//    if (robot_)
//    {
//        // Update the robot sensors
//        std::vector<ObjectConstPtr> sensors;
//        std::vector<geo::Pose3D> sensor_poses;

//        robot_->getSensors(sensors, sensor_poses);

//        for(unsigned int i = 0; i < sensors.size(); ++i)
//        {
//            sensors[i]->sense(*world_, sensor_poses[i]);
//        }
//    }

//    world_ = world_updated;

    // - - - - - - - - - - - - - - -

    WorldPtr world_updated;

    // collect all update requests
    std::vector<PluginContainerPtr> plugins_with_requests;
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        const PluginContainerPtr& c = it->second;

        if (c->updateRequest())
        {
            if (!world_updated)
                world_updated = boost::make_shared<World>(*world_);   // Create a world copy

            world_updated->update(*c->updateRequest());
            plugins_with_requests.push_back(c);
        }
    }

    if (world_updated)
        world_ = world_updated; // Swap to updated world (if something changed)

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
