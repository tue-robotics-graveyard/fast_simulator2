#ifndef SIMULATOR_ROS_ROBOT_PLUGIN_H_
#define SIMULATOR_ROS_ROBOT_PLUGIN_H_

#include "fast_simulator2/plugin.h"

#include <kdl/tree.hpp>
#include <fast_simulator2/update_request.h>

struct Joint
{
    double position;
    KDL::Segment segment; // calculates the joint pose
    sim::LUId transform_id;
};

class ROSRobotPlugin : public sim::Plugin
{

public:

    ROSRobotPlugin();

    void configure(tue::Configuration config, const sim::LUId& obj_id);

    void process(const sim::World& world, const sim::LUId& obj_id, double dt, sim::UpdateRequest& req);

private:

    sim::UpdateRequest init_update_request_;

    KDL::Tree tree_;

    /// Joint positions
    std::map<std::string, Joint> joints_;

    bool updateJoint(const std::string& name, double pos, sim::UpdateRequest& req);

};

#endif
