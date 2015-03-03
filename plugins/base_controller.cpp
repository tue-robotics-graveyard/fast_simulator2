#include "base_controller.h"

#include <ed/world_model.h>
#include <ed/uuid.h>

#include <ros/time.h>

// ----------------------------------------------------------------------------------------------------

BaseController::BaseController()
{
}

// ----------------------------------------------------------------------------------------------------

void BaseController::configure(tue::Configuration config, const sim::LUId& obj_id)
{

}

// ----------------------------------------------------------------------------------------------------

void BaseController::process(const ed::WorldModel& world, const sim::LUId& obj_id, double dt, ed::UpdateRequest& req)
{
    std::cout << "BaseController::process (" << obj_id.id << ")" << std::endl;

    // Get ROS current time
    ros::Time time = ros::Time::now();

    geo::Pose3D t;
    if (world.calculateTransform(ed::UUID(obj_id.id), "world", time.toSec(), t))
    {
        std::cout << t << std::endl;
    }

//    req.addTransform(world.rootId(), obj_id, geo::Pose3D(2, 0, 0));
}

SIM_REGISTER_PLUGIN(BaseController)
