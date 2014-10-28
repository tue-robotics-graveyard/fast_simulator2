#include "base_controller.h"

#include <tue/simulator/world.h>
#include <tue/simulator/object.h>
#include <tue/simulator/update_request.h>

// ----------------------------------------------------------------------------------------------------

BaseController::BaseController()
{
}

// ----------------------------------------------------------------------------------------------------

void BaseController::configure(tue::Configuration config)
{

}

// ----------------------------------------------------------------------------------------------------

void BaseController::process(const sim::World& world, const sim::Object& obj, double dt, sim::UpdateRequest& req)
{
    std::cout << "BaseController::process" << std::endl;

    std::cout << obj.id() << ": " << obj.pose() << std::endl;

    req.setPose(obj.id(), geo::Pose3D::identity());
}

SIM_REGISTER_PLUGIN(BaseController)
