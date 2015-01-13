#include "ros_robot_plugin.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// For loading urdf model
#include <fstream>
#include <sstream>

// Finding ros packages
#include <ros/package.h>

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
//
//                                   RESOLVE FUNCTION PARSING
//
// ----------------------------------------------------------------------------------------------------

bool executeResolvefunction(const std::vector<std::string>& args, std::string& result, std::stringstream& error)
{
    if (args[0] == "rospkg" && args.size() == 2)
    {
        result = ros::package::getPath(args[1]);
        if (result.empty())
        {
            error << "ROS package '" << args[1] << "' unknown.";
            return false;
        }

        return true;
    }
    else if (args[0] == "env" && (args.size() == 2 || args.size() == 3))
    {
        char* env_value;
        env_value = getenv(args[1].c_str());
        if (env_value == 0)
        {
            if (args.size() == 3)
            {
                // Default value
                result = args[2];
                return true;
            }

            error << "Environment variable '" << args[1] << "' unknown.";
            return false;
        }

        result = env_value;
        return true;
    }

    error << "Unknown resolve function: '" << args[0] << "' with " << args.size() - 1 << " arguments.";

    return false;
}

// ----------------------------------------------------------------------------------------------------

bool parseResolveFunction(const std::string& str, std::size_t& i, std::string& result, std::stringstream& error)
{
    std::vector<std::string> args;
    args.push_back("");

    for(; i < str.size();)
    {
        char c = str[i];

        if (c == '$' && (i + 1) < str.size() && str[i + 1] == '(')
        {
            // Skip "$("
            i += 2;

            std::string arg;
            if (!parseResolveFunction(str, i, arg, error))
                return false;

            args.back() += arg;
            continue;
        }
        else if (c == ')')
        {
            ++i;

            // Check if last argument is empty. If so, remove it
            if (args.back().empty())
                args.pop_back();

            // Check if args are empty. Is so, return false
            if (args.empty())
            {
                error << "Empty resolve function.";
                return false;
            }

            return executeResolvefunction(args, result, error);
        }
        else if (c == ' ')
        {
            if (!args.back().empty())
                args.push_back("");
            ++i;
        }
        else
        {
            args.back() += c;
            ++i;
        }
    }

    error << "Missing ')'.";
    return false;
}

// ----------------------------------------------------------------------------------------------------

bool resolve(const std::string& str, std::string& result, std::stringstream& error)
{
    std::size_t i = 0;

    while(i < str.size())
    {
        std::size_t i_sign = str.find("$(", i);
        if (i_sign == std::string::npos)
        {
            result += str.substr(i);
            return true;
        }

        result += str.substr(i, i_sign - i);

        // Skip until after "$("
        i = i_sign + 2;

        std::string subresult;
        if (!parseResolveFunction(str, i, subresult, error))
            return false;

        result += subresult;
    }

    error << "Missing ')'.";
    return false;
}

// ----------------------------------------------------------------------------------------------------
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
        std::string urdf_file_resolved;
        std::stringstream resolve_error;
        if (!resolve(urdf_file, urdf_file_resolved, resolve_error))
        {
            config.addError("Could not resolve 'urdf' value '" + urdf_file + "': " + resolve_error.str());
            return;
        }

        std::ifstream f(urdf_file_resolved.c_str());

        if (!f.is_open())
        {
            config.addError("Could not load URDF description. File not found: '" + urdf_file_resolved + "'.");
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
