#ifndef ED_SIMULATOR_ROBOT_H_
#define ED_SIMULATOR_ROBOT_H_

#include <tue/config/configuration.h>

#include "types.h"
#include <map>
#include <geolib/datatypes.h>

namespace sim
{

class Robot
{

public:

    Robot();

    virtual ~Robot();

    void configure(tue::Configuration config);

    void addLink(const std::string& name, const geo::Pose3D& pose)
    {
        link_positions_[name] = pose;
    }

private:

    /// Link positions relative to the robot pose
    std::map<std::string, geo::Pose3D> link_positions_;

};

}

#endif
