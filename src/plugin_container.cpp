#include "plugin_container.h"

#include "tue/simulator/update_request.h"
#include "tue/simulator/world.h"


#include <ros/rate.h> // TODO: make own implementation

namespace sim
{

// --------------------------------------------------------------------------------

PluginContainer::PluginContainer()
    : class_loader_(0), cycle_duration_(0.1), loop_frequency_(10), stop_(false), step_finished_(true), t_last_update_(0)
{
}

// --------------------------------------------------------------------------------

PluginContainer::~PluginContainer()
{
    stop_ = true;
    thread_->join();
    plugin_.reset();
    delete class_loader_;
}

// --------------------------------------------------------------------------------

PluginPtr PluginContainer::loadPlugin(const std::string plugin_name, const std::string& lib_filename,
                tue::Configuration config, std::string& error)
{    
    // Load the library
    delete class_loader_;
    class_loader_ = new class_loader::ClassLoader(lib_filename);

    // Create plugin
    class_loader_->loadLibrary();
    std::vector<std::string> classes = class_loader_->getAvailableClasses<sim::Plugin>();

    if (classes.empty())
    {
        error += "Could not find any plugins in '" + class_loader_->getLibraryPath() + "'.";
    } else if (classes.size() > 1)
    {
        error += "Multiple plugins registered in '" + class_loader_->getLibraryPath() + "'.";
    } else
    {
        plugin_ = class_loader_->createInstance<Plugin>(classes.front());
        if (plugin_)
        {
            // Configure plugin
            plugin_->configure(config);
            plugin_->name_ = plugin_name;

            config.value("_object", object_id_.id, tue::OPTIONAL);

            return plugin_;
        }
    }

    return PluginPtr();
}

// --------------------------------------------------------------------------------

void PluginContainer::runThreaded()
{
    thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PluginContainer::run, this)));
}

// --------------------------------------------------------------------------------

void PluginContainer::run()
{
    ros::Rate r(loop_frequency_);
    while(!stop_)
    {
        step();
        r.sleep();
    }
}

// --------------------------------------------------------------------------------

void PluginContainer::step()
{
    // If we still have an update_request, it means the request is not yet handled,
    // so we have to skip this cycle (and wait until the world model has handled it)
    {
        boost::lock_guard<boost::mutex> lg(mutex_update_request_);
        if (update_request_)
            return;
    }

    // Check if there is a new world. If so replace the current one with the new one
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        if (world_new_)
        {
            world_current_ = world_new_;
            world_new_.reset();
        }
    }

    if (world_current_)
    {
        UpdateRequestPtr update_request(new UpdateRequest);

        plugin_->process(*world_current_, cycle_duration_, *update_request);

        if (!object_id_.id.empty())
            plugin_->process(*world_current_, object_id_, cycle_duration_, *update_request);

        // If the received update_request was not empty, set it
        if (!update_request->empty())
            update_request_ = update_request;
    }
}

// --------------------------------------------------------------------------------

void PluginContainer::stop()
{
    stop_ = true;
    thread_->join();
    plugin_.reset();
}

// --------------------------------------------------------------------------------

} // end namespace sim

