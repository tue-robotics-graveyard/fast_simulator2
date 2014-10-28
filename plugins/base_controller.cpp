#include "base_controller.h"

#include <tue/simulator/world.h>
#include <tue/simulator/object.h>
#include <tue/simulator/update_request.h>

// ----------------------------------------------------------------------------------------------------

BaseController::BaseController()
{
}

// ----------------------------------------------------------------------------------------------------

BaseController::~BaseController()
{
}

// ----------------------------------------------------------------------------------------------------

void BaseController::configure(tue::Configuration config)
{

}

// ----------------------------------------------------------------------------------------------------

void BaseController::process(const sim::World& world, double dt, sim::UpdateRequest& req)
{
    sim::ObjectConstPtr base = world.object("amigo");

    std::cout << world.objects.size() << std::endl;

    if (base)
        std::cout << base->pose() << std::endl;
    std::cout << "BaseController::process" << std::endl;
}

SIM_REGISTER_PLUGIN(BaseController)
