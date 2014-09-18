#include "depth_sensor.h"

#include "world.h"

#include <geolib/Shape.h>
#include <geolib/Box.h>

#include <opencv2/highgui/highgui.hpp>

// ROS
#include <rgbd/ros/conversions.h>
#include <sensor_msgs/Image.h>

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

    rgbd::convert(camera_, cam_info_depth_);
    rgbd::convert(camera_, cam_info_rgb_);
}

// ----------------------------------------------------------------------------------------------------

DepthSensor::~DepthSensor()
{
}

// ----------------------------------------------------------------------------------------------------

void DepthSensor::sense(const World& world, const geo::Pose3D& sensor_pose) const
{
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

    cv::imshow("depth", depth_image / 8);
    cv::waitKey(3);
}

}
