#include "tue/simulator/world.h"

#include "tue/simulator/object.h"

namespace sim
{

// ----------------------------------------------------------------------------------------------------

World::World()
{
}

// ----------------------------------------------------------------------------------------------------

World::~World()
{
}

// ----------------------------------------------------------------------------------------------------

ObjectId World::addObject(const ObjectConstPtr& obj)
{
    std::map<UUID, unsigned int>::const_iterator it = id_to_index_.find(obj->id());
    if (it == id_to_index_.end())
    {
        id_to_index_[obj->id()] = objects_.size();
        objects_.push_back(obj);
        return ObjectId(obj->id(), objects_.size() - 1);
    }
    else
    {
        objects_[it->second] = obj;
        return ObjectId(obj->id(), it->second);
    }
}

// ----------------------------------------------------------------------------------------------------

bool World::removeObject(const ObjectId& id)
{
    int index = getIndex(id);
    if (index < 0)
        return false;

    objects_[index] = objects_[objects_.size() - 1];
    objects_.pop_back();
}

// ----------------------------------------------------------------------------------------------------

ObjectConstPtr World::object(const ObjectId& id) const
{
    int index = getIndex(id);
    id.index = index;
    if (index >= 0)
        return objects_[index];
    else
        return ObjectConstPtr();
}

// ----------------------------------------------------------------------------------------------------

int World::getIndex(const ObjectId& id) const
{
    int id_index = id.index;
    if (id_index >= 0 && id_index < objects_.size() && id.id == objects_[id_index]->id())
        return id_index;
    else
    {
        std::map<UUID, unsigned int>::const_iterator it = id_to_index_.find(id.id);
        if (it == id_to_index_.end())
            return -1;
        return it->second;
    }
}

}
