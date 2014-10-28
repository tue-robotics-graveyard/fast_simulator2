#ifndef ED_SIMULATOR_PLUGIN_H_
#define ED_SIMULATOR_PLUGIN_H_

#include <class_loader/class_loader.h>
#define SIM_REGISTER_PLUGIN(Derived) CLASS_LOADER_REGISTER_CLASS(Derived, sim::Plugin)

#include "types.h"

#include <tue/config/configuration.h>

namespace sim
{

class Plugin
{

    friend class PluginContainer;

public:

    Plugin();

    virtual ~Plugin();

    virtual void configure(tue::Configuration config) {}

    virtual void initialize() {}

    virtual void process(const World& world, double dt, UpdateRequest& req) {}

    const std::string& name() const { return name_; }

private:

    std::string name_;

};

} // end namespace sim

#endif
