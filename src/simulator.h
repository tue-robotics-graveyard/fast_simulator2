#ifndef ED_SIMULATOR_SIMULATOR_H_
#define ED_SIMULATOR_SIMULATOR_H_

#include "tue/simulator/types.h"

#include <vector>
#include <map>

#include <tue/config/configuration.h>

namespace sim
{

class Simulator
{

public:

    Simulator();

    virtual ~Simulator();

    void configure(tue::Configuration config);

    void step(double dt, std::vector<ObjectConstPtr>& changed_objects);

    void addObject(const ObjectConstPtr& object);

    PluginContainerPtr loadPlugin(const std::string plugin_name, const std::string& lib_filename,
                                  tue::Configuration config, std::string& error);

    WorldConstPtr world() const { return world_; }

private:

    RobotPtr robot_;

    WorldPtr world_;

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::map<std::string, PluginContainerPtr> plugin_containers_;

};

}

#endif
