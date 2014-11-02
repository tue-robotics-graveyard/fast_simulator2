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

LUId World::addTransform(const TransformConstPtr& t)
{
    return transforms_.add(t);
}

// ----------------------------------------------------------------------------------------------------

bool World::getTransform(const LUId& source, const LUId& target) const
{
//    ObjectConstPtr t = object(source);
//    t->
}

}
