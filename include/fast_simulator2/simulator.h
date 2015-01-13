#ifndef FAST_SIMULATOR2_SIMULATOR_H_
#define FAST_SIMULATOR2_SIMULATOR_H_

#include "fast_simulator2/types.h"

#include <ed/types.h>

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

    PluginContainerPtr loadPlugin(const std::string plugin_name, const std::string& lib_filename,
                                  tue::Configuration config, std::string& error);

    const ed::WorldModelConstPtr& world() const { return world_; }

private:

    RobotPtr robot_;

//    WorldConstPtr world_;

    ed::WorldModelConstPtr world_;

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::map<std::string, PluginContainerPtr> plugin_containers_;

    // Models
    std::map<std::string, std::string> models_;

    void createObject(tue::Configuration config, UpdateRequest& req);

    void createObject(LUId parent_id, tue::Configuration config, ed::UpdateRequest& req);

};

}

#endif
