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

class DepthSensorRenderResult : public geo::RenderResult {

public:

    DepthSensorRenderResult(cv::Mat& z_buffer, int width, int height)
                : geo::RenderResult(width, height), z_buffer_(z_buffer)
    {
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer_.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            z_buffer_.at<float>(y, x) = depth;
        }
    }

protected:

    cv::Mat& z_buffer_;

};

// ----------------------------------------------------------------------------------------------------

DepthSensorPlugin::DepthSensorPlugin() : render_rgb_(false), render_depth_(false)
{
}

// ----------------------------------------------------------------------------------------------------

void DepthSensorPlugin::configure(tue::Configuration config, const sim::LUId& obj_id)
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
    geo::Pose3D p;
    if (world.getTransform(obj_id, sim::LUId("world"), p))
    {
        std::cout << "KINECT POSE: " << p << std::endl;
    }
    else
    {
        std::cout << "NO KINECT POSE FOUND" << std::endl;
    }

    // Get ROS current time
    ros::Time time = ros::Time::now();

    cv::Mat depth_image;
    cv::Mat rgb_image;

    if (render_rgb_)
    {
        rgb_image = cv::Mat(rgb_height_, rgb_width_, CV_8UC3, cv::Scalar(255,255,255));
    }

    if (render_depth_)
    {
        // Calculate sensor pose in geolib frame
//        geo::Pose3D geolib_pose = sensor_pose * geo::Pose3D(0, 0, 0, 3.1415, 0, 0);

        depth_image = cv::Mat(depth_height_, depth_width_, CV_32FC1, 0.0);
        for(std::vector<sim::ObjectConstPtr>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
        {
            const sim::ObjectConstPtr& obj = *it;
            geo::ShapeConstPtr shape = obj->shape();

            if (shape)
            {
                geo::Pose3D rel_pose;
                // Calculate pose of object relative to camera
                if (world.getTransform(obj_id, obj->id(), rel_pose))
                {
                    std::cout << obj->id() << ": " << rel_pose << std::endl;

                    // Correction for geolib frame
//                    rel_pose = rel_pose * geo::Pose3D(0, 0, 0, 3.1415, 0, 0);

                    // Set render options
                    geo::RenderOptions opt;
                    opt.setMesh(shape->getMesh(), rel_pose);
                    DepthSensorRenderResult res(depth_image, depth_width_, depth_height_);

                    // Render
                    depth_rasterizer_.render(opt, res);
                }
            }
        }
    }

    if (!pubs_depth_.empty())
    {
        // Convert depth image to ROS message
        sensor_msgs::Image depth_image_msg;
        rgbd::convert(depth_image, depth_image_msg);
        depth_image_msg.header.stamp = time;
        depth_image_msg.header.frame_id = depth_frame_id_;

        // Publish image
        for(std::vector<ros::Publisher>::const_iterator it = pubs_depth_.begin(); it != pubs_depth_.end(); ++it)
            it->publish(depth_image_msg);
    }

    if (!pubs_cam_info_depth_.empty())
    {
        // Convert camera info to ROS message
        sensor_msgs::CameraInfo cam_info_depth;
        rgbd::convert(depth_rasterizer_, cam_info_depth);
        cam_info_depth.header.stamp = time;
        cam_info_depth.header.frame_id = depth_frame_id_;

        // Publish camera info
        for(std::vector<ros::Publisher>::const_iterator it = pubs_cam_info_depth_.begin(); it != pubs_cam_info_depth_.end(); ++it)
            it->publish(cam_info_depth);
    }

    if (!pubs_rgb_.empty())
    {
        // Convert rgb image to ROS message
        sensor_msgs::Image rgb_image_msg;
        rgbd::convert(rgb_image, rgb_image_msg);
        rgb_image_msg.header.stamp = time;
        rgb_image_msg.header.frame_id = rgb_frame_id_;

        // Publish image
        for(std::vector<ros::Publisher>::const_iterator it = pubs_rgb_.begin(); it != pubs_rgb_.end(); ++it)
            it->publish(rgb_image_msg);
    }

    if (!pubs_cam_info_rgb_.empty())
    {
        // Convert camera info to ROS message
        sensor_msgs::CameraInfo cam_info_rgb;
        rgbd::convert(depth_rasterizer_, cam_info_rgb);
        cam_info_rgb.header.stamp = time;
        cam_info_rgb.header.frame_id = rgb_frame_id_;

        // Publish camera info
        for(std::vector<ros::Publisher>::const_iterator it = pubs_cam_info_rgb_.begin(); it != pubs_cam_info_rgb_.end(); ++it)
            it->publish(cam_info_rgb);
    }

    if (!pubs_rgbd_.empty())
    {
        rgbd::Image image(rgb_image, depth_image, depth_rasterizer_, rgb_frame_id_, time.toSec());

        rgbd::RGBDMsg msg;
        msg.version = 2;

        // Set RGB storage type
        rgbd::RGBStorageType rgb_type = rgbd::RGB_STORAGE_JPG;
        if (!rgb_image.data)
            rgb_type = rgbd::RGB_STORAGE_NONE;

        rgbd::DepthStorageType depth_type = rgbd::DEPTH_STORAGE_PNG;
        if (!depth_image.data)
            depth_type = rgbd::DEPTH_STORAGE_NONE;

        std::stringstream stream;
        tue::serialization::OutputArchive a(stream);
        rgbd::serialize(image, a, rgb_type, depth_type);
        tue::serialization::convert(stream, msg.rgb);

        for(std::vector<ros::Publisher>::const_iterator it = pubs_rgbd_.begin(); it != pubs_rgbd_.end(); ++it)
            it->publish(msg);
    }
}

SIM_REGISTER_PLUGIN(DepthSensorPlugin)
