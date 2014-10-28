#ifndef ED_SIMULATOR_WORLD_H_
#define ED_SIMULATOR_WORLD_H_

#include "types.h"
#include <map>

namespace sim
{

class World
{

public:

    World();

    virtual ~World();

    std::map<UUID, ObjectConstPtr> objects;

};

}

#endif
