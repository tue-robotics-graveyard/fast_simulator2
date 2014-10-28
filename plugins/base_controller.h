#ifndef SIMULATOR_BASE_CONTROLLER_H_
#define SIMULATOR_BASE_CONTROLLER_H_

#include "tue/simulator/plugin.h"

class BaseController : public sim::Plugin
{

public:

    BaseController();

    virtual ~BaseController();

    void configure(tue::Configuration config);

    void process(const sim::World& world, double dt, sim::UpdateRequest& req);

private:

};

#endif
