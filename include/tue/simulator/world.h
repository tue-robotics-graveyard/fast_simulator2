#ifndef ED_SIMULATOR_WORLD_H_
#define ED_SIMULATOR_WORLD_H_

#include "types.h"
#include <map>
#include <vector>

namespace sim
{

class World
{

public:

    World();

    virtual ~World();

    ObjectId addObject(const ObjectConstPtr& obj);

    bool removeObject(const ObjectId& id);

    ObjectConstPtr object(const ObjectId& id) const;

    inline const std::vector<ObjectConstPtr>& objects() const { return objects_; }

private:

    std::vector<ObjectConstPtr> objects_;

    std::map<UUID, unsigned int> id_to_index_;

    int getIndex(const ObjectId& id) const;

};

}

#endif
