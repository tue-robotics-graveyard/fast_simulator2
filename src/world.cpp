#include "tue/simulator/world.h"
#include "tue/simulator/object.h"
#include "tue/simulator/update_request.h"

namespace sim
{

// ----------------------------------------------------------------------------------------------------

World::World()
{
    // Add root object
    ObjectPtr root = boost::make_shared<Object>("world");
    root_id_ = addObject(root);
}

// ----------------------------------------------------------------------------------------------------

World::~World()
{
}

// ----------------------------------------------------------------------------------------------------

void World::update(const UpdateRequest& req)
{
    // Add objects
    for(std::vector<std::pair<LUId, ObjectConstPtr> >::const_iterator it = req.objects.begin(); it != req.objects.end(); ++it)
    {
        objects_.insert(it->first, it->second);
    }

    // Add transforms
    for(std::vector<TransformConstPtr>::const_iterator it = req.transforms.begin(); it != req.transforms.end(); ++it)
    {
        const TransformConstPtr& t = *it;
        const LUId& child_id = t->child;
        const LUId& parent_id = t->parent;

        const ObjectConstPtr& parent = object(parent_id);
        const ObjectConstPtr& child = object(child_id);

        if (parent && child)
        {
            LUId transform_id = addTransform(t);

            ObjectPtr parent_new = boost::make_shared<Object>(*parent);
            ObjectPtr child_new = boost::make_shared<Object>(*child);

            parent_new->addTransform(child_id, transform_id);
            child_new->setParent(parent_id, transform_id);

            objects_.insert(parent_id, parent_new);
            objects_.insert(child_id, child_new);

        }
    }

    // Update transforms
    for(std::vector<std::pair<LUId, geo::Pose3D> >::const_iterator it = req.transform_ups.begin(); it != req.transform_ups.end(); ++it)
    {
        updateTransform(it->first, it->second);
    }
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

bool World::getTransform(const LUId& source_id, const LUId& target_id, geo::Pose3D& pose) const
{
    const ObjectConstPtr& source = object(source_id);
    if (!source)
        return false;

    const ObjectConstPtr& target = object(target_id);
    if (!target)
        return false;

    // Make a set of ancestors;
    std::set<int> ancestors;
    for(const LUId* n = &source_id; true; )
    {
        ancestors.insert(n->index);
        const ObjectConstPtr& obj = object(*n);
        if (!obj)
            break; // object not found (should never happen)

        n = &obj->parent();
        if (n->id.empty())
            break; // No parent
    }

    // Find common ancestor
    const LUId* a_common = &target_id;
    while (true)
    {
        if (ancestors.find(a_common->index) != ancestors.end())
            break;  // Found it!

        const ObjectConstPtr& obj = object(*a_common);
        if (!obj)
            return false; // object not found (should never happen)

        a_common = &obj->parent();
        if (a_common->id.empty())
            return false; // No common ancestor found
    }

    // Calculate transforms from source to common ancestor
    geo::Pose3D t1 = geo::Pose3D::identity();
    for(const LUId* n = &source_id; true; )
    {
        if (n->index == a_common->index)
            break;

        const ObjectConstPtr& obj = object(*n);
        const TransformConstPtr t = transforms_.get(obj->parentTransform());
        if (!t)
            return false; // Should never happen

        t1 = t1 * t->pose;

        n = &obj->parent();
    }

    // Calculate transforms from target to common ancestor
    geo::Pose3D t2 = geo::Pose3D::identity();
    for(const LUId* n = &target_id; true; )
    {
        if (n->index == a_common->index)
            break;

        const ObjectConstPtr& obj = object(*n);
        const TransformConstPtr t = transforms_.get(obj->parentTransform());
        if (!t)
            return false; // Should never happen

        t2 = t2 * t->pose;

        n = &obj->parent();
    }

    pose = t1.inverseTimes(t2);

    return true;
}

}
