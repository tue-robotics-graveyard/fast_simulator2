#include "ros_robot_plugin.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// For loading urdf model
#include <fstream>
#include <sstream>

// ----------------------------------------------------------------------------------------------------

bool JointRelation::calculateTransform(const ed::Time& t, geo::Pose3D& tf) const
{
    // Calculate joint pose for this joint position
    KDL::Frame pose_kdl = segment_.pose(joint_pos_);

    // Convert to geolib transform
    tf.R = geo::Matrix3(pose_kdl.M.data);
    tf.t = geo::Vector3(pose_kdl.p.data);

    return true;
}

// ----------------------------------------------------------------------------------------------------

void ROSRobotPlugin::constructRobot(const ed::UUID& parent_id, const KDL::SegmentMap::const_iterator& it_segment, ed::UpdateRequest& req)
{
    const KDL::Segment& segment = it_segment->second.segment;

    // Child ID is the segment (link) name
    ed::UUID child_id = robot_name_ + "/" + segment.getName();

    // Set the entity type (robot_link)
    req.setType(child_id, "robot_link");

    // Create a joint relation and add id
    boost::shared_ptr<JointRelation> r(new JointRelation(segment));
    r->setJointPosition(0);
    req.setRelation(parent_id, child_id, r);

    // Generate relation info that will be used to update the relation
    RelationInfo& rel_info = joint_name_to_rel_info_[segment.getJoint().getName()];
    rel_info.parent_id = parent_id;
    rel_info.child_id = child_id;
    rel_info.r_idx = ed::INVALID_IDX;
    rel_info.relation = r;

    // Recursively add all children
    const std::vector<KDL::SegmentMap::const_iterator>& children = it_segment->second.children;
    for (unsigned int i = 0; i < children.size(); i++)
        constructRobot(child_id, children[i], req);
}

// ----------------------------------------------------------------------------------------------------

ROSRobotPlugin::ROSRobotPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void ROSRobotPlugin::configure(tue::Configuration config, const sim::LUId& obj_id)
{   
    robot_name_ = obj_id.id;

    std::string urdf_file;
    if (config.value("urdf", urdf_file))
    {
        std::ifstream f(urdf_file.c_str());

        if (!f.is_open())
        {
            config.addError("Could not load URDF description. File not found: '" + urdf_file + "'.");
            return;
        }

        std::stringstream buffer;
        buffer << f.rdbuf();
        std::string urdf_xml = buffer.str();

        urdf::Model robot_model;

        if (!robot_model.initString(urdf_xml))
        {
            config.addError("Could not initialize robot model");
            return;
        }

        if (!kdl_parser::treeFromString(urdf_xml, tree_))
        {
            config.addError("Could not initialize tree object");
            return;

        }

        constructRobot(obj_id.id, tree_.getRootSegment(), init_update_request_);
    }

    if (config.readArray("joints"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            double pos;
            if (config.value("name", name) && config.value("position", pos))
            {
                if (!updateJoint(name, pos, init_update_request_))
                    config.addError("No such joint in URDF model: '" + name + "'.");
            }
        }

        config.endArray();
    }
}

// ----------------------------------------------------------------------------------------------------

bool ROSRobotPlugin::updateJoint(const std::string& name, double pos, ed::UpdateRequest& req)
{
    std::map<std::string, RelationInfo>::iterator it = joint_name_to_rel_info_.find(name);
    if (it == joint_name_to_rel_info_.end())
        return false;

    RelationInfo& info = it->second;

    // Make a copy of the relation
    boost::shared_ptr<JointRelation> r(new JointRelation(*info.relation));
    r->setJointPosition(pos);
    req.setRelation(info.parent_id, info.child_id, r);
    info.relation = r;

    return true;
}

// ----------------------------------------------------------------------------------------------------

void ROSRobotPlugin::process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req)
{
    if (!init_update_request_.empty())
    {
        // Send the update request created in the configuration
        req = init_update_request_;

        // Clear the request
        init_update_request_ = ed::UpdateRequest();

        return;
    }
}

SIM_REGISTER_PLUGIN(ROSRobotPlugin)
