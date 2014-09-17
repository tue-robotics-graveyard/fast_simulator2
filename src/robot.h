#ifndef ED_SIMULATOR_ROBOT_H_
#define ED_SIMULATOR_ROBOT_H_

#include <tue/config/configuration.h>

#include "types.h"
#include <map>
#include <geolib/datatypes.h>

#include <kdl/tree.hpp>

namespace sim
{

class Robot
{

public:

    Robot();

    virtual ~Robot();

    void configure(tue::Configuration config);

    void setLink(const std::string& name, const geo::Pose3D& pose) { link_positions_[name] = pose; }

    void setJoint(const std::string& name, double pos) { joint_positions_[name] = pos; }

    void getSensors(std::vector<ObjectConstPtr>& sensors, std::vector<geo::Pose3D>& poses);

private:

    KDL::Tree tree_;

    /// Joint positions
    std::map<std::string, double> joint_positions_;

    /// Link positions relative to the robot pose
    std::map<std::string, geo::Pose3D> link_positions_;

    std::map<std::string, ObjectConstPtr> sensors_;

    void calculateLinkPositions();

    void calculateLinkPositions(const geo::Pose3D& parent_pose, const KDL::SegmentMap::const_iterator& segment);




};

}

#endif
