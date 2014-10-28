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

    ObjectConstPtr object(const UUID& id) const
    {
        std::map<UUID, ObjectConstPtr>::const_iterator it = objects.find(id);
        if (it == objects.end())
            return ObjectConstPtr();
        else
            return it->second;
    }

    std::map<UUID, ObjectConstPtr> objects;

};

}

#endif
