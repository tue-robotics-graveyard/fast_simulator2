#ifndef SIMULATOR_BASE_CONTROLLER_H_
#define SIMULATOR_BASE_CONTROLLER_H_

#include "fast_simulator2/plugin.h"

class BaseController : public sim::Plugin
{

public:

    BaseController();

    void configure(tue::Configuration config, const sim::LUId& obj_id);

    void process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req);

private:

};

#endif
