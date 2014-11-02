#ifndef ED_SIMULATOR_ID_MAP_H_
#define ED_SIMULATOR_ID_MAP_H_

#include "types.h"
#include <map>
#include <vector>

namespace sim
{

template<typename T>
class IDMap
{

public:

    IDMap()
    {
    }

    // ----------------------------------------------------------------------------------------------------

    ~IDMap()
    {
    }

    // ----------------------------------------------------------------------------------------------------

    inline LUId add(const T& obj)
    {
        std::map<UUId, unsigned int>::const_iterator it = id_to_index_.find(obj->id());
        if (it == id_to_index_.end())
        {
            id_to_index_[obj->id()] = objects_.size();
            objects_.push_back(obj);
            return LUId(obj->id(), objects_.size() - 1);
        }
        else
        {
            objects_[it->second] = obj;
            return LUId(obj->id(), it->second);
        }
    }

    // ----------------------------------------------------------------------------------------------------

    inline bool remove(const LUId& id)
    {
        int index = getIndex(id);
        if (index < 0)
            return false;

        objects_[index] = objects_[objects_.size() - 1];
        objects_.pop_back();
    }

    // ----------------------------------------------------------------------------------------------------

    inline T get(const LUId& id) const
    {
        int index = getIndex(id);
        id.index = index;
        if (index >= 0)
            return objects_[index];
        else
            return ObjectConstPtr();
    }

    inline const std::vector<T>& getAll() const { return objects_; }

private:

    std::vector<T> objects_;

    std::map<UUId, unsigned int> id_to_index_;

    // ----------------------------------------------------------------------------------------------------

    inline int getIndex(const LUId& id) const
    {
        int id_index = id.index;
        if (id_index >= 0 && id_index < objects_.size() && id.id == objects_[id_index]->id())
            return id_index;
        else
        {
            std::map<UUId, unsigned int>::const_iterator it = id_to_index_.find(id.id);
            if (it == id_to_index_.end())
                return -1;
            return it->second;
        }
    }
};

}

#endif
