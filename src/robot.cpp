#include "robot.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// For loading urdf model
#include <fstream>
#include <sstream>

namespace sim
{

// ----------------------------------------------------------------------------------------------------

Robot::Robot()
{
}

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
}

// ----------------------------------------------------------------------------------------------------

void Robot::configure(tue::Configuration config)
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

        calculateLinkPositions();
    }

    if (config.readArray("joints"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            double pos;
            if (config.value("name", name) && config.value("position", pos))
            {
                std::map<std::string, double>::iterator it = joint_positions_.find(name);
                if (it == joint_positions_.end())
                {
                    config.addError("No such joint in URDF model: '" + name + "'.");
                }
                else
                {
                    it->second = pos;
                }
            }
        }

        calculateLinkPositions();

        config.endArray();
    }

    if (config.readArray("sensors"))
    {
        while(config.nextArrayItem())
        {
            std::string type, link;
            if (config.value("type", type) && config.value("link", link))
            {
                std::map<std::string, geo::Pose3D>::const_iterator it = link_positions_.find(link);
                if (it == link_positions_.end())
                {
                    config.addError("No such link in URDF model: '" + link + "'.");
                }
                else
                {
                    const geo::Pose3D& sensor_pose = it->second;
                    std::cout << "Sensor " << type << ": " << sensor_pose << std::endl;
                }
            }
        }

        config.endArray();
    }
}

// ----------------------------------------------------------------------------------------------------

void Robot::calculateLinkPositions()
{
    calculateLinkPositions(geo::Pose3D::identity(), tree_.getRootSegment());
}

// ----------------------------------------------------------------------------------------------------

void Robot::calculateLinkPositions(const geo::Pose3D& parent_pose, const KDL::SegmentMap::const_iterator& segment)
{
    const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;
    for (unsigned int i = 0; i < children.size(); i++)
    {
        const KDL::Segment& child_kdl = children[i]->second.segment;

        // Find joint position, or set default (0)
        double joint_pos;
        std::map<std::string, double>::iterator it_joint = joint_positions_.find(child_kdl.getJoint().getName());
        if (it_joint == joint_positions_.end())
        {
            joint_pos = 0;
            joint_positions_[child_kdl.getJoint().getName()] = joint_pos;
        }
        else
        {
            joint_pos = it_joint->second;
        }

        // Get the pose relative to the parent (with joint position 0)
        KDL::Frame rel_pose_kdl = child_kdl.pose(joint_pos);
        geo::Pose3D rel_pose(geo::Matrix3(rel_pose_kdl.M.data), geo::Vector3(rel_pose_kdl.p.data));

        // Calculate link position relative to the robot
        geo::Pose3D abs_pose = parent_pose * rel_pose;

        setLink(child_kdl.getName(), abs_pose);

        // recursively add segments
        calculateLinkPositions(abs_pose, children[i]);
    }
}

// ----------------------------------------------------------------------------------------------------


}
