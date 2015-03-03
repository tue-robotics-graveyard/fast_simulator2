#ifndef SIMULATOR_ROS_ROBOT_PLUGIN_H_
#define SIMULATOR_ROS_ROBOT_PLUGIN_H_

#include "fast_simulator2/plugin.h"

#include <kdl/tree.hpp>
#include <ed/update_request.h>
#include <ed/relation.h>

#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

// ----------------------------------------------------------------------------------------------------

class JointRelation : public ed::Relation
{

public:

    JointRelation(const KDL::Segment& segment) : segment_(segment) {}

    ed::Time latestTime() const { return ed::Time(0); }  // TODO

    bool calculateTransform(const ed::Time& t, geo::Pose3D& tf) const;

    void setJointPosition(float joint_pos) { joint_pos_ = joint_pos; }

    float jointPosition() const { return joint_pos_; }

private:

    float joint_pos_;
    KDL::Segment segment_; // calculates the joint pose

};

// ----------------------------------------------------------------------------------------------------

struct RelationInfo
{
    std::string joint_name;
    ed::UUID parent_id;
    ed::UUID child_id;
    ed::Idx r_idx;
    boost::shared_ptr<const JointRelation> relation;
};

// ----------------------------------------------------------------------------------------------------

struct JointGroup
{
    ros::Publisher pub;
    std::vector<RelationInfo*> joints;
};

// ----------------------------------------------------------------------------------------------------

class ROSRobotPlugin : public sim::Plugin
{

public:

    ROSRobotPlugin();

    ~ROSRobotPlugin();

    void configure(tue::Configuration config, const sim::LUId& obj_id);

    void process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req);

private:

    std::string robot_name_;

    ed::UpdateRequest init_update_request_;

    KDL::Tree tree_;


    /// Joint positions
    std::map<std::string, RelationInfo> joint_name_to_rel_info_;

    bool updateJoint(const std::string& name, double pos, ed::UpdateRequest& req);

    void constructRobot(const ed::UUID& parent_id, const KDL::SegmentMap::const_iterator& it_segment, ed::UpdateRequest& req);


    /// Publising

    std::vector<JointGroup> joint_groups_;

    void publishJointStates();


    /// TF Publishing

    tf::TransformBroadcaster* tf_broadcaster_;

};

#endif
