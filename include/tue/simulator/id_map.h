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

    inline void insert(const LUId& id, const T& obj)
    {
        int index = getIndex(id);
        if (index >= 0)
        {
            items_[index] = obj;
            id.index = index;
        }
        else
        {
            index = items_.size();
            id_to_index_[id.id] = index;
            items_.push_back(obj);
            ids_.push_back(id.id);
            id.index = index;
        }
    }

    // ----------------------------------------------------------------------------------------------------

    inline bool remove(const LUId& id)
    {
        int index = getIndex(id);
        if (index < 0)
            return false;

        items_[index] = items_[items_.size() - 1];
        items_.pop_back();
    }

    // ----------------------------------------------------------------------------------------------------

    inline const T& get(const LUId& id) const
    {
        int index = getIndex(id);
        id.index = index;
        if (index >= 0)
            return items_[index];
        else
            return null_;
    }

    inline const std::vector<T>& getAll() const { return items_; }

private:

    T null_;

    std::vector<UUId> ids_;

    std::vector<T> items_;

    std::map<UUId, unsigned int> id_to_index_;

    // ----------------------------------------------------------------------------------------------------

    inline int getIndex(const LUId& id) const
    {
        int id_index = id.index;
        if (id_index >= 0 && id_index < items_.size() && id.id == ids_[id_index])
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
