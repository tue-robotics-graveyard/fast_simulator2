#include "simulator.h"

#include "tue/simulator/world.h"
#include "tue/simulator/object.h"
#include "tue/simulator/update_request.h"

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
            cfg.data().add(config.data());

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

void Simulator::createObject(tue::Configuration config, UpdateRequest& req)
{
    std::string id, type;
    if (!config.value("id", id) || !config.value("type", type))
        return;

    std::map<std::string, std::string>::const_iterator it = models_.find(type);
    if (it != models_.end())
    {
        config = expandObjectConfig(models_, config);

        // Add object
        ObjectPtr e = boost::make_shared<Object>(id);
        e->setType(type);
        req.addObject(e);
    }

    tue::Configuration params;
    if (config.readGroup("parameters"))
    {
        params = config;
        config.endGroup();
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
                plugin_cfg.data().add(params.data());

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

// ----------------------------------------------------------------------------------------------------

void Simulator::createObject(const LUId& parent_id, tue::Configuration config, UpdateRequest& req)
{
    std::string id, type;
    if (!config.value("id", id) || !config.value("type", type))
        return;

    geo::Pose3D pose = geo::Pose3D::identity();
    if (config.readGroup("pose", tue::REQUIRED))
    {
        config.value("x", pose.t.x);
        config.value("y", pose.t.y);
        config.value("z", pose.t.z);

        double roll = 0, pitch = 0, yaw = 0;
        config.value("roll", roll, tue::OPTIONAL);
        config.value("pitch", pitch, tue::OPTIONAL);
        config.value("yaw", yaw, tue::OPTIONAL);
        pose.R.setRPY(roll, pitch, yaw);

        config.endGroup();
    }
    else
        return;

    std::map<std::string, std::string>::const_iterator it = models_.find(type);
    if (it != models_.end())
    {
        config = expandObjectConfig(models_, config);

        // Add object
        ObjectPtr e = boost::make_shared<Object>(id);
        e->setType(type);
        LUId obj_id = req.addObject(e);

        req.addTransform(parent_id, obj_id, pose);
    }
    else
    {
        ed::models::NewEntityConstPtr e = ed::models::create(type, config, id);
        if (e)
            addObjectRecursive(req, e, geo::Pose3D::identity());
        else
            config.addError("Unknown object type: '" + type + "'.");
    }

    tue::Configuration params;
    if (config.readGroup("parameters"))
    {
        params = config;
        config.endGroup();
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
                plugin_cfg.data().add(params.data());

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

    //  Composition
    if (config.readArray("children"))
    {
        while(config.nextArrayItem())
        {
            createObject(id, config, req);
        }
        config.endArray();
    }
}

// ----------------------------------------------------------------------------------------------------

void Simulator::configure(tue::Configuration config)
{
    UpdateRequest req;

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
            createObject(world_->rootId(), config, req);
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

            const UpdateRequestConstPtr& req = c->updateRequest();
            if (!req->object_configs.empty())
            {
                UpdateRequest req2;
                for(std::vector<tue::Configuration>::const_iterator it2 = req->object_configs.begin(); it2 != req->object_configs.end(); ++it2)
                {
                    createObject(*it2, req2);
                }
                world_updated->update(req2);
            }

            world_updated->update(*req);
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
