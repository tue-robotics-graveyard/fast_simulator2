#include "depth_sensor.h"

#include "world.h"

#include <geolib/Shape.h>
#include <geolib/Box.h>

#include <opencv2/highgui/highgui.hpp>

// ROS
#include <rgbd/ros/conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/node_handle.h>

namespace sim
{

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

DepthSensor::DepthSensor() : width_(640), height_(480)
{
    // TODO: read from configuration
    camera_.setOpticalTranslation(0, 0);
    camera_.setOpticalCenter(320.5, 240.5);
    camera_.setFocalLengths(558, 558);

    ros::NodeHandle nh;

    pubs_rgb_.push_back(nh.advertise<sensor_msgs::Image>("rgb_topic", 10));
    pubs_depth_.push_back(nh.advertise<sensor_msgs::Image>("depth_topic", 10));
    pubs_cam_info_rgb_.push_back(nh.advertise<sensor_msgs::CameraInfo>("rgb_info_topic", 10));
    pubs_cam_info_depth_.push_back(nh.advertise<sensor_msgs::CameraInfo>("depth_info_topic", 10));
}

// ----------------------------------------------------------------------------------------------------

DepthSensor::~DepthSensor()
{
}

// ----------------------------------------------------------------------------------------------------

void DepthSensor::sense(const World& world, const geo::Pose3D& sensor_pose) const
{
    // Get ROS current time
    ros::Time time = ros::Time::now();

    // Calculate sensor pose in geolib frame
    geo::Pose3D geolib_pose = sensor_pose * geo::Pose3D(0, 0, 0, 3.1415, 0, 0);

    cv::Mat depth_image = cv::Mat(height_, width_, CV_32FC1, 0.0);
    for(std::map<UUID, ObjectConstPtr>::const_iterator it = world.objects.begin(); it != world.objects.end(); ++it)
    {
        const ObjectConstPtr& obj = it->second;
        geo::ShapeConstPtr shape = obj->shape();

        if (shape)
        {
            // Calculate pose of object relative to camera
            geo::Pose3D rel_pose = geolib_pose.inverse() * obj->pose();

            // Set render options
            geo::RenderOptions opt;
            opt.setMesh(shape->getMesh(), rel_pose);
            DepthSensorRenderResult res(depth_image, width_, height_);

            // Render
            camera_.render(opt, res);
        }
    }

    // Convert depth image to ROS message
    sensor_msgs::Image depth_image_msg;
    rgbd::convert(depth_image, depth_image_msg);

    // Convert camera info to ROS message
    sensor_msgs::CameraInfo cam_info_depth, cam_info_rgb;
    rgbd::convert(camera_, cam_info_depth);
    rgbd::convert(camera_, cam_info_rgb);

    // Set time stamps
    cam_info_rgb.header.stamp = time;
    cam_info_depth.header.stamp = time;
    depth_image_msg.header.stamp = time;

    // Set frame ids
    cam_info_rgb.header.frame_id = rgb_frame_id_;
    cam_info_depth.header.frame_id = depth_frame_id_;
    depth_image_msg.header.frame_id = depth_frame_id_;

    for(std::vector<ros::Publisher>::const_iterator it = pubs_cam_info_rgb_.begin(); it != pubs_cam_info_rgb_.end(); ++it)
        it->publish(cam_info_rgb);

    for(std::vector<ros::Publisher>::const_iterator it = pubs_cam_info_depth_.begin(); it != pubs_cam_info_depth_.end(); ++it)
        it->publish(cam_info_depth);

    for(std::vector<ros::Publisher>::const_iterator it = pubs_depth_.begin(); it != pubs_depth_.end(); ++it)
        it->publish(depth_image_msg);
}

}
