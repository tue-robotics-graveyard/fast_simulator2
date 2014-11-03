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
    LUId id(t->id());
    transforms_.insert(id, t);
    return id;
}

// ----------------------------------------------------------------------------------------------------

void World::updateTransform(const LUId& id, const geo::Pose3D& pose)
{
    const TransformConstPtr& t = transforms_.get(id);
    if (t)
    {
        // TODO: should be possible to do this more efficiently (not having to copy parent and child id each time)
        TransformPtr t_new = boost::make_shared<Transform>(t->parent, t->child, pose);
        transforms_.insert(id, t_new);
    }
}

// ----------------------------------------------------------------------------------------------------

bool World::getTransform(const LUId& source, const LUId& target, geo::Pose3D& t) const
{
//    ObjectConstPtr t = object(source);
//    t->
}

}
