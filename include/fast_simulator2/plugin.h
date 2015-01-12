#ifndef FAST_SIMULATOR2_PLUGIN_H_
#define FAST_SIMULATOR2_PLUGIN_H_

#include <class_loader/class_loader.h>
#define SIM_REGISTER_PLUGIN(Derived) CLASS_LOADER_REGISTER_CLASS(Derived, sim::Plugin)

#include "fast_simulator2/types.h"

#include <tue/config/configuration.h>
#include <ed/types.h>

namespace sim
{

class Plugin
{

    friend class PluginContainer;

public:

    Plugin() {}

    virtual void configure(tue::Configuration config, const sim::LUId& obj_id) {}

    virtual void initialize() {}

    virtual void process(const ed::WorldModel& world, double dt, ed::UpdateRequest& req) {}

    virtual void process(const ed::WorldModel& world, const LUId& obj_id, double dt, ed::UpdateRequest& req) {}

    const std::string& name() const { return name_; }

private:

    std::string name_;

};

} // end namespace sim

#endif
