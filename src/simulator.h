#ifndef ED_SIMULATOR_SIMULATOR_H_
#define ED_SIMULATOR_SIMULATOR_H_

#include "tue/simulator/types.h"

#include <vector>

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

    WorldConstPtr world() const { return world_; }

private:

    RobotPtr robot_;

    WorldPtr world_;

};

}

#endif
