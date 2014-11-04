#include "ros_robot_plugin.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// For loading urdf model
#include <fstream>
#include <sstream>

#include <tue/simulator/object.h>
#include <tue/simulator/world.h>

void constructRobot(const std::string& ns, const sim::LUId& parent_id, const KDL::SegmentMap::const_iterator& it_segment,
                    std::map<std::string, Joint>& joints, sim::UpdateRequest& req)
{
    const KDL::Segment& segment = it_segment->second.segment;

    // Convert KDL segment to sim object and add it
    sim::ObjectPtr obj = boost::make_shared<sim::Object>(ns + "/" + segment.getName());
    req.addObject(obj);

    // Create Joint object
    Joint joint;
    joint.position = 0;
    joint.segment = segment;

    // Calculate pose with default joint position (0)
    KDL::Frame pose_kdl = segment.pose(joint.position);
    geo::Pose3D pose(geo::Matrix3(pose_kdl.M.data), geo::Vector3(pose_kdl.p.data));

    // Add the transform
    joint.transform_id = req.addTransform(parent_id, obj->id(), pose);

    // Set the joint info
    joints[segment.getJoint().getName()] = joint;

    // Recursively add all children
    const std::vector<KDL::SegmentMap::const_iterator>& children = it_segment->second.children;
    for (unsigned int i = 0; i < children.size(); i++)
    {
        const KDL::Segment& child_kdl = children[i]->second.segment;

        // recursively add segments
        constructRobot(ns, obj->id(), children[i], joints, req);
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

        constructRobot(obj_id.id, obj_id, tree_.getRootSegment(), joints_, init_update_request_);
    }

    if (config.readArray("sensors"))
    {
        while(config.nextArrayItem())
        {
            sim::LUId link_id, sensor_id;
            if (config.value("id", sensor_id.id) & config.value("link", link_id.id))
            {
                sensor_id.id = obj_id.id + "/" + sensor_id.id;
                config.setValue("id", sensor_id.id);

                link_id.id = obj_id.id + "/" + link_id.id;
                sim::LUId sensor_id = init_update_request_.addObject(config);
                init_update_request_.addTransform(link_id, sensor_id, geo::Pose3D::identity());
            }
        }
        config.endArray();
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

void printTransformTree(const sim::World& world, const sim::LUId& obj_id, const std::string& indent = "")
{
    const sim::ObjectConstPtr& obj = world.object(obj_id);

    std::cout << indent << obj_id.id << std::endl;

    for(std::map<sim::LUId, sim::LUId>::const_iterator it = obj->transforms().begin(); it != obj->transforms().end(); ++it)
    {
        printTransformTree(world, it->first, indent + "  ");
    }
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

//    printTransformTree(world, world.rootId());


//    geo::Pose3D p;
//    if (world.getTransform(sim::LUId("amigo/hand_right"), sim::LUId("amigo/base_link"), p))
//    {
//        std::cout << p << std::endl;
//    }
}

SIM_REGISTER_PLUGIN(ROSRobotPlugin)
