#include "ros_robot_plugin.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// For loading urdf model
#include <fstream>
#include <sstream>

#include <fast_simulator2/object.h>
#include <fast_simulator2/world.h>

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
    r->setCacheSize(joint_cache_size_);
    r->insert(0, 0);
    req.setRelation(parent_id, child_id, r);

    // Generate relation info that will be used to update the relation
    RelationInfo& rel_info = joint_name_to_rel_info_[segment.getJoint().getName()];
    rel_info.parent_id = parent_id;
    rel_info.child_id = child_id;
    rel_info.r_idx = ed::INVALID_IDX;
    rel_info.last_rel = r;

    // Recursively add all children
    const std::vector<KDL::SegmentMap::const_iterator>& children = it_segment->second.children;
    for (unsigned int i = 0; i < children.size(); i++)
        constructRobot(child_id, children[i], req);
}

// ----------------------------------------------------------------------------------------------------

ROSRobotPlugin::ROSRobotPlugin() : joint_cache_size_(10)
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

    if (config.readArray("sensors"))
    {
        while(config.nextArrayItem())
        {
            sim::LUId link_id, sensor_id;
            if (config.value("id", sensor_id.id) & config.value("link", link_id.id))
            {
                sensor_id.id = robot_name_ + "/" + sensor_id.id;
                config.setValue("id", sensor_id.id);

                link_id.id = robot_name_ + "/" + link_id.id;
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

void ROSRobotPlugin::process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req)
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
