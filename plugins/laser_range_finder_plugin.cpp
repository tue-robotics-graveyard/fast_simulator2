#include "laser_range_finder_plugin.h"

#include <geolib/Shape.h>

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

#include <ed/world_model.h>
#include <ed/world_model/transform_crawler.h>
#include <ed/uuid.h>
#include <ed/entity.h>

// ----------------------------------------------------------------------------------------------------

LaserRangeFinderPlugin::LaserRangeFinderPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

LaserRangeFinderPlugin::~LaserRangeFinderPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void LaserRangeFinderPlugin::configure(tue::Configuration config, const sim::LUId& obj_id)
{
    int num_beams;
    double min_angle, max_angle, min_range, max_range;

    config.value("num_beams", num_beams);
    config.value("min_angle", min_angle);
    config.value("max_angle", max_angle);
    config.value("min_range", min_range);
    config.value("max_range", max_range);

    // Set LRF parameters
    lrf_.setNumBeams(num_beams);
    lrf_.setAngleLimits(min_angle, max_angle);
    lrf_.setRangeLimits(min_range, max_range);

    // Make sure ROS is initialized
    if (!ros::isInitialized())
         ros::init(ros::M_string(), "simulator", ros::init_options::NoSigintHandler);

    // Set-up communication (ROS publisher)
    std::string topic, frame_id;
    if (config.value("topic", topic) & config.value("frame_id", frame_id))
    {
        ros::NodeHandle nh;
        pub_ = nh.advertise<sensor_msgs::LaserScan>(topic, 10);

        scan_.header.frame_id = frame_id;
        scan_.angle_min = lrf_.getAngleMin();
        scan_.angle_max = lrf_.getAngleMax();
        scan_.angle_increment = lrf_.getAngleIncrement();
        scan_.time_increment = 0;
        scan_.scan_time = 0;
        scan_.range_min = lrf_.getRangeMin();
        scan_.range_max = lrf_.getRangeMax();
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserRangeFinderPlugin::process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req)
{
    std::vector<double> ranges(lrf_.getNumBeams(), 0);

    for(ed::world_model::TransformCrawler tc(world, obj_id.id, world.latestTime()); tc.hasNext(); tc.next())
    {
        const ed::EntityConstPtr& e = tc.entity();

        if (e->shape())
        {
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), tc.transform());

            geo::LaserRangeFinder::RenderResult res(ranges);
            lrf_.render(opt, res);
        }
    }

    // Make sure ranges in scan message is correct size
    if (scan_.ranges.size() != lrf_.getNumBeams())
        scan_.ranges.resize(lrf_.getNumBeams());

    // Copy ranges to scan message
    for(unsigned int i = 0; i < ranges.size(); ++i)
        scan_.ranges[i] = ranges[i];

    // Stamp with current ROS time
    scan_.header.stamp = ros::Time::now();

    pub_.publish(scan_);
}

SIM_REGISTER_PLUGIN(LaserRangeFinderPlugin)
