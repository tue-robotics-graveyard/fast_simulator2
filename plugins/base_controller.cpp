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
    std::cout << base->pose() << std::endl;
}
