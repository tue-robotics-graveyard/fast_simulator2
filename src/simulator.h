#ifndef ED_SIMULATOR_SIMULATOR_H_
#define ED_SIMULATOR_SIMULATOR_H_

#include "types.h"

#include <map>
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

private:

    std::map<UUID, ObjectConstPtr> objects_;

};

}

#endif
