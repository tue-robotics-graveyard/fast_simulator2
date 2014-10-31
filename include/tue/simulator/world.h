#ifndef ED_SIMULATOR_WORLD_H_
#define ED_SIMULATOR_WORLD_H_

#include "object.h"
#include "id_map.h"

namespace sim
{

class World
{

public:

    World();

    virtual ~World();

    inline LUId addObject(const ObjectConstPtr& obj) { return objects_.add(obj->id(), obj); }

    inline bool removeObject(const LUId& id) { return objects_.remove(id); }

    inline ObjectConstPtr object(const LUId& id) const { return objects_.get(id); }

    inline const std::vector<ObjectConstPtr>& objects() const { return objects_.getAll(); }

private:

    IDMap<ObjectConstPtr> objects_;

};

}

#endif
