#ifndef ED_SIMULATOR_TYPES_H_
#define ED_SIMULATOR_TYPES_H_

#include <string>
#include <boost/shared_ptr.hpp>

namespace sim
{

typedef std::string UUID;

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

}

#endif
