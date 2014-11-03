#include "depth_sensor_plugin.h"

#include "tue/simulator/world.h"

#include <geolib/Shape.h>
#include <geolib/Box.h>

#include <opencv2/highgui/highgui.hpp>

#include <rgbd/Image.h>
#include <rgbd/serialization.h>
#include <rgbd/RGBDMsg.h>
#include <tue/serialization/conversions.h>

// ROS
#include <rgbd/ros/conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/node_handle.h>

// ----------------------------------------------------------------------------------------------------

DepthSensorPlugin::DepthSensorPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void DepthSensorPlugin::configure(tue::Configuration config)
{
    if (config.readGroup("rgb"))
    {
        config.value("width", rgb_width_);
        config.value("height", rgb_height_);

        render_rgb_ = true;

        config.endGroup();
    }

    if (config.readGroup("depth"))
    {
        config.value("width", depth_width_);
        config.value("height", depth_height_);

        double fx, fy;
        config.value("fx", fx);
        config.value("fy", fy);

        depth_rasterizer_.setOpticalTranslation(0, 0);
        depth_rasterizer_.setOpticalCenter((depth_width_ + 1) / 2, (depth_height_ + 1) / 2);
        depth_rasterizer_.setFocalLengths(fx, fy);

        render_depth_ = true;

        config.endGroup();
    }

    if (!ros::isInitialized())
         ros::init(ros::M_string(), "simulator", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;

    if (config.readArray("topics"))
    {
        while(config.nextArrayItem())
        {
            std::string rgbd_topic;
            if (config.value("rgbd", rgbd_topic, tue::OPTIONAL))
                pubs_rgbd_.push_back(nh.advertise<rgbd::RGBDMsg>(rgbd_topic, 10));

            std::string depth_topic;
            if (config.value("depth", depth_topic, tue::OPTIONAL))
                pubs_depth_.push_back(nh.advertise<sensor_msgs::Image>(depth_topic, 10));

            std::string depth_info_topic;
            if (config.value("depth_info", depth_info_topic, tue::OPTIONAL))
                pubs_cam_info_depth_.push_back(nh.advertise<sensor_msgs::CameraInfo>(depth_info_topic, 10));

            std::string rgb_topic;
            if (config.value("rgb", rgb_topic, tue::OPTIONAL))
                pubs_rgb_.push_back(nh.advertise<sensor_msgs::Image>(rgb_topic, 10));

            std::string rgb_info_topic;
            if (config.value("rgb_info", rgb_info_topic, tue::OPTIONAL))
                pubs_cam_info_rgb_.push_back(nh.advertise<sensor_msgs::CameraInfo>(rgb_info_topic, 10));
        }

        config.endArray();
    }

    config.value("frame_id", rgb_frame_id_);
    depth_frame_id_ = rgb_frame_id_;
}

// ----------------------------------------------------------------------------------------------------

void DepthSensorPlugin::process(const sim::World& world, const sim::LUId& obj_id, double dt, sim::UpdateRequest& req)
{
    std::cout << "DepthSensorPlugin::process (" << obj_id.id << ")" << std::endl;

    std::cout << rgb_width_ << std::endl;

    geo::Pose3D t;
    if (world.getTransform(obj_id, world.rootId(), t))
    {
        std::cout << t << std::endl;
    }

//    req.addTransform(world.rootId(), obj_id, geo::Pose3D(2, 0, 0));
}

SIM_REGISTER_PLUGIN(DepthSensorPlugin)
