#ifndef FAST_SIMULATOR2_LASER_RANGE_FINDER_H_
#define FAST_SIMULATOR2_LASER_RANGE_FINDER_H_

#include "fast_simulator2/plugin.h"

#include <geolib/sensors/LaserRangeFinder.h>
#include <ros/publisher.h>
#include <sensor_msgs/LaserScan.h>

class LaserRangeFinderPlugin : public sim::Plugin
{

public:

    LaserRangeFinderPlugin();

    virtual ~LaserRangeFinderPlugin();

    void configure(tue::Configuration config, const sim::LUId& obj_id);

    void process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req);

private:

    geo::LaserRangeFinder lrf_;

    ros::Publisher pub_;

    sensor_msgs::LaserScan scan_;

};

#endif
