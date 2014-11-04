#include "tue/simulator/update_request.h"
#include "tue/simulator/object.h"

namespace sim
{

// ----------------------------------------------------------------------------------------------------

LUId UpdateRequest::addObject(const ObjectConstPtr& obj)
{
    objects.push_back(std::pair<LUId, ObjectConstPtr>(obj->id(), obj));
    return obj->id();
}

// ----------------------------------------------------------------------------------------------------

LUId UpdateRequest::addObject(tue::Configuration config)
{
    object_configs.push_back(config);

    LUId id;
    config.value("id", id.id);
    return id;
}

}
