#ifndef ED_SIMULATOR_PLUGIN_CONTAINER_H_
#define ED_SIMULATOR_PLUGIN_CONTAINER_H_

#include "tue/simulator/types.h"

#include <boost/thread.hpp>

namespace sim
{

class PluginContainer
{

public:

    PluginContainer();

    virtual ~PluginContainer();

    void setPlugin(PluginPtr plugin, const std::string& name);

    PluginPtr plugin() const { return plugin_; }

    void runThreaded();

    void stop();

    const std::string& name() const { return name_; }

    UpdateRequestConstPtr updateRequest() const
    {
        boost::lock_guard<boost::mutex> lg(mutex_update_request_);
        return update_request_;
    }

    void clearUpdateRequest()
    {
        boost::lock_guard<boost::mutex> lg(mutex_update_request_);
        update_request_.reset();
    }

    void setWorld(const WorldConstPtr& world)
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        world_new_ = world;
    }

    void setLoopFrequency(double freq) { loop_frequency_ = freq; }

protected:

    PluginPtr plugin_;

    std::string name_;

    bool stop_;

    // 1.0 / cycle frequency
    double cycle_duration_;

    double loop_frequency_;

    mutable boost::mutex mutex_update_request_;

    UpdateRequestPtr update_request_;

    boost::shared_ptr<boost::thread> thread_;

    bool step_finished_;

    double t_last_update_;

    mutable boost::mutex mutex_world_;

    WorldConstPtr world_new_;

    WorldConstPtr world_current_;

    void step();

    void run();

};

} // end namespace sim

#endif
