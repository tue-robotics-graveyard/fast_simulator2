#ifndef ED_SIMULATOR_SIMULATOR_PLUGIN_H_
#define ED_SIMULATOR_SIMULATOR_PLUGIN_H_

#include <ed/plugin.h>

#include <ed/types.h>

// Simulator
#include "simulator.h"

// Communication
#include <ros/callback_queue.h>
#include <ros/service_server.h>
#include <ros/publisher.h>

// Configuration
#include <tue/config/configuration.h>

// Services
#include <ed/SetEntity.h>

class SimulatorPlugin : public ed::Plugin
{

public:

    SimulatorPlugin();

    virtual ~SimulatorPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    // Simulator

    sim::Simulator simulator_;

    // World model update

    const ed::WorldModel* world_model_;

    ed::UpdateRequest* update_req_;

    // Communication

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_set_entity_;


    bool srvSetEntity(ed::SetEntity::Request& req, ed::SetEntity::Response& res);

};

#endif
