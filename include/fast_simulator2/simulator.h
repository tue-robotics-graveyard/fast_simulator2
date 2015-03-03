#ifndef FAST_SIMULATOR2_SIMULATOR_H_
#define FAST_SIMULATOR2_SIMULATOR_H_

#include "fast_simulator2/types.h"

#include <ed/types.h>
#include <ed/models/model_loader.h>

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

    void step(double dt);

    PluginContainerPtr loadPlugin(const std::string plugin_name, const std::string& lib_filename,
                                  tue::Configuration config, std::string& error);

    const ed::WorldModelConstPtr& world() const { return world_; }

    void addPluginPath(const std::string& path) { plugin_paths_.push_back(path); }

private:

    ed::WorldModelConstPtr world_;

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::map<std::string, PluginContainerPtr> plugin_containers_;

    // Models
    std::map<std::string, std::string> models_;

    std::string model_path_;

    ed::models::ModelLoader model_loader_;

    void createObject(LUId parent_id, tue::Configuration config, ed::UpdateRequest& req);

    tue::config::DataPointer loadModelData(const std::string& type);

    std::string getFullLibraryPath(const std::string& lib);

};

}

#endif
