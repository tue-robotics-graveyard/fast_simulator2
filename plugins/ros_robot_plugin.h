#ifndef SIMULATOR_ROS_ROBOT_PLUGIN_H_
#define SIMULATOR_ROS_ROBOT_PLUGIN_H_

#include "fast_simulator2/plugin.h"

#include <kdl/tree.hpp>
#include <ed/update_request.h>
#include <ed/time_cache.h>
#include <ed/relation.h>

// ----------------------------------------------------------------------------------------------------

class JointRelation : public ed::Relation
{

public:

    JointRelation(const KDL::Segment& segment) : segment_(segment) {}

    ed::Time latestTime() const { return joint_pos_cache_.latestTime(); }

    bool calculateTransform(const ed::Time& t, geo::Pose3D& tf) const;

    void insert(const ed::Time& t, float joint_pos) { joint_pos_cache_.insert(t, joint_pos); }

    inline unsigned int size() const { return joint_pos_cache_.size(); }

    void setCacheSize(unsigned int n) { joint_pos_cache_.setMaxSize(n); }

private:

    ed::TimeCache<float> joint_pos_cache_;
    KDL::Segment segment_; // calculates the joint pose

};


// ----------------------------------------------------------------------------------------------------

struct RelationInfo
{
    ed::UUID parent_id;
    ed::UUID child_id;
    ed::Idx r_idx;
    boost::shared_ptr<const JointRelation> last_rel;
};

// ----------------------------------------------------------------------------------------------------

class ROSRobotPlugin : public sim::Plugin
{

public:

    ROSRobotPlugin();

    void configure(tue::Configuration config, const sim::LUId& obj_id);

    void process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req);

private:

    std::string robot_name_;

    ed::UpdateRequest init_update_request_;

    KDL::Tree tree_;

    unsigned int joint_cache_size_;

    /// Joint positions
    std::map<std::string, RelationInfo> joint_name_to_rel_info_;

    bool updateJoint(const std::string& name, double pos, sim::UpdateRequest& req);

    void constructRobot(const ed::UUID& parent_id, const KDL::SegmentMap::const_iterator& it_segment, ed::UpdateRequest& req);

};

#endif
