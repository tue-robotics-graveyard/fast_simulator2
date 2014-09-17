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

}

#endif
