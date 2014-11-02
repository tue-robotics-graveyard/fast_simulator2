#ifndef ED_SIMULATOR_TYPES_H_
#define ED_SIMULATOR_TYPES_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <geolib/datatypes.h>

namespace sim
{

typedef std::string UUId;

class Object;
typedef boost::shared_ptr<Object> ObjectPtr;
typedef boost::shared_ptr<const Object> ObjectConstPtr;

class Robot;
typedef boost::shared_ptr<Robot> RobotPtr;
typedef boost::shared_ptr<const Robot> RobotConstPtr;

class World;
typedef boost::shared_ptr<World> WorldPtr;
typedef boost::shared_ptr<const World> WorldConstPtr;

class UpdateRequest;
typedef boost::shared_ptr<UpdateRequest> UpdateRequestPtr;
typedef boost::shared_ptr<const UpdateRequest> UpdateRequestConstPtr;

class Plugin;
typedef boost::shared_ptr<Plugin> PluginPtr;
typedef boost::shared_ptr<const Plugin> PluginConstPtr;

class PluginContainer;
typedef boost::shared_ptr<PluginContainer> PluginContainerPtr;
typedef boost::shared_ptr<const PluginContainer> PluginContainerConstPtr;

struct LUId
{
    LUId(const UUId& id_ = "", int index_ = -1) : id(id_), index(index_) {}

    UUId id;
    mutable int index;
};

struct Transform
{
    Transform() {}

    Transform(const LUId& parent_, const LUId& child_, const geo::Pose3D& pose_)
        : parent(parent_), child(child_), pose(pose_)
    {
        // TODO: better id generation
        id_ = parent.id + "-" + child.id;
    }

    const UUId& id() const { return id_; }

    LUId parent;
    LUId child;
    geo::Pose3D pose;

private:

    UUId id_;
};
typedef boost::shared_ptr<Transform> TransformPtr;
typedef boost::shared_ptr<const Transform> TransformConstPtr;

}

#endif
