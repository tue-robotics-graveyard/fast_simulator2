#include "ros_robot_plugin.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// For loading urdf model
#include <fstream>
#include <sstream>

#include <ros/node_handle.h>
#include <geolib/ros/tf_conversions.h>

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

    if (segment.getJoint().getName() == "NoName")
    {
        std::cout << "Do something!" << std::endl;
    }



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
    rel_info.joint_name = segment.getJoint().getName();
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

ROSRobotPlugin::ROSRobotPlugin() : tf_broadcaster_(0)
{
}

// ----------------------------------------------------------------------------------------------------

ROSRobotPlugin::~ROSRobotPlugin()
{
    delete tf_broadcaster_;
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

    // Init ROS (if needed)
    if (!ros::isInitialized())
         ros::init(ros::M_string(), "simulator", ros::init_options::NoSigintHandler);

    std::string measurements_topic;
    if (config.value("measurements_topic", measurements_topic))
    {
        ros::NodeHandle nh;

        joint_groups_.push_back(JointGroup());
        JointGroup& jg = joint_groups_.back();
        jg.pub = nh.advertise<sensor_msgs::JointState>(measurements_topic, 100);

        // Add all joints to the group
        for(std::map<std::string, RelationInfo>::iterator it = joint_name_to_rel_info_.begin(); it != joint_name_to_rel_info_.end(); ++it)
        {
            jg.joints.push_back(&(it->second));
        }
    }

    int publish_tf;
    if(config.value("publish_tf", publish_tf, tue::OPTIONAL) && publish_tf)
        tf_broadcaster_ = new tf::TransformBroadcaster;
    else
        delete tf_broadcaster_;
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

    publishJointStates();
}

// ----------------------------------------------------------------------------------------------------

void ROSRobotPlugin::publishJointStates()
{    
    ros::Time ros_time = ros::Time::now();

    for(std::vector<JointGroup>::const_iterator it = joint_groups_.begin(); it != joint_groups_.end(); ++it)
    {
        const JointGroup& jg = *it;

        sensor_msgs::JointState js_msg;
        js_msg.header.stamp = ros_time;
        for(std::vector<RelationInfo*>::const_iterator it_joint = jg.joints.begin(); it_joint != jg.joints.end(); ++it_joint)
        {
            const RelationInfo* info = *it_joint;
            js_msg.name.push_back(info->joint_name);
            js_msg.position.push_back(info->relation->jointPosition());
        }

        jg.pub.publish(js_msg);
    }

    if (tf_broadcaster_)
    {
        std::vector<tf::StampedTransform> transforms;
        for(std::map<std::string, RelationInfo>::const_iterator it = joint_name_to_rel_info_.begin(); it != joint_name_to_rel_info_.end(); ++it)
        {
            const RelationInfo& rel = it->second;

            // Calculate joint pose for this joint position
            geo::Pose3D pose;
            rel.relation->calculateTransform(rel.relation->latestTime(), pose);

            transforms.push_back(tf::StampedTransform());
            tf::StampedTransform& pose_tf = transforms.back();

            geo::convert(pose, pose_tf);
            pose_tf.frame_id_ = rel.parent_id.str();
            pose_tf.child_frame_id_ = rel.child_id.str();
            pose_tf.stamp_ = ros_time;
        }

        tf_broadcaster_->sendTransform(transforms);
    }
}

SIM_REGISTER_PLUGIN(ROSRobotPlugin)
