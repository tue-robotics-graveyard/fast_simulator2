#include "ros_robot_plugin.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// For loading urdf model
#include <fstream>
#include <sstream>

void constructRobot(const sim::LUId& parent_id, const KDL::SegmentMap::const_iterator& segment,
                    std::map<std::string, Joint>& joints, sim::UpdateRequest& req)
{
    const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;
    for (unsigned int i = 0; i < children.size(); i++)
    {
        const KDL::Segment& child_kdl = children[i]->second.segment;

        Joint joint;
        joint.position = 0;
        joint.segment = child_kdl;

        KDL::Frame pose_kdl = child_kdl.pose(joint.position);
        geo::Pose3D pose(geo::Matrix3(pose_kdl.M.data), geo::Vector3(pose_kdl.p.data));

        req.addObject(...);

        joint.transform_id = req.addTransform(parent_id, child_kdl.getJoint().getName(), pose);

        joints[child_kdl.getJoint().getName()] = joint;

        // recursively add segments
        constructRobot(child_kdl.getName(), children[i], joints, req);
    }
}

// ----------------------------------------------------------------------------------------------------

ROSRobotPlugin::ROSRobotPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void ROSRobotPlugin::configure(tue::Configuration config, const sim::LUId& obj_id)
{
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

        constructRobot(obj_id, tree_.getRootSegment(), joints_, init_update_request_);
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

    std::cout << init_update_request_.objects.size() << std::endl;
}

// ----------------------------------------------------------------------------------------------------

bool ROSRobotPlugin::updateJoint(const std::string& name, double pos, sim::UpdateRequest& req)
{
    std::map<std::string, Joint>::iterator it = joints_.find(name);
    if (it == joints_.end())
        return false;

    Joint& joint = it->second;
    joint.position = pos;
    KDL::Frame pose_kdl = joint.segment.pose(pos);
    geo::Pose3D pose(geo::Matrix3(pose_kdl.M.data), geo::Vector3(pose_kdl.p.data));

    req.updateTransform(joint.transform_id, pose);

    return true;
}

// ----------------------------------------------------------------------------------------------------

void ROSRobotPlugin::process(const sim::World& world, const sim::LUId& obj_id, double dt, sim::UpdateRequest& req)
{
    if (!init_update_request_.empty())
    {
        // Send the update request created in the configuration
        req = init_update_request_;

        // Clear the request
        init_update_request_ = sim::UpdateRequest();

        return;
    }


    std::cout << "ROS ROBOT " << obj_id.id << std::endl;
}

SIM_REGISTER_PLUGIN(ROSRobotPlugin)
