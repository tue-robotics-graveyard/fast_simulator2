#ifndef ED_SIMULATOR_SIMULATOR_H_
#define ED_SIMULATOR_SIMULATOR_H_

#include "types.h"

#include <map>
#include <vector>

namespace sim
{

class Simulator
{

public:

    Simulator();

    virtual ~Simulator();

    void step(double dt, std::vector<ObjectConstPtr>& changed_objects);

    void addObject(const ObjectConstPtr& object);

private:

    std::map<UUID, ObjectConstPtr> objects_;

};

}

#endif
