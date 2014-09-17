#include "simulator_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/models/loader.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <geolib/ros/msg_conversions.h>

// Simulator
#include "object.h"

// ----------------------------------------------------------------------------------------------------

SimulatorPlugin::SimulatorPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

SimulatorPlugin::~SimulatorPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void SimulatorPlugin::configure(tue::Configuration config)
{

}

// ----------------------------------------------------------------------------------------------------

void SimulatorPlugin::initialize()
{
    ros::NodeHandle nh;
    ros::AdvertiseServiceOptions opt_set_entity =
            ros::AdvertiseServiceOptions::create<ed::SetEntity>(
                "/ed/sim/set_entity", boost::bind(&SimulatorPlugin::srvSetEntity, this, _1, _2), ros::VoidPtr(), &cb_queue_);
    srv_set_entity_ = nh.advertiseService(opt_set_entity);
}

// ----------------------------------------------------------------------------------------------------

void SimulatorPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;
    update_req_ = &req;
    cb_queue_.callAvailable();

    // Step the simulator
    std::vector<sim::ObjectConstPtr> changed_objects;
    simulator_.step(0.01, changed_objects);

    // Update all changed objects in the world model
    for(std::vector<sim::ObjectConstPtr>::const_iterator it = changed_objects.begin(); it != changed_objects.end(); ++it)
    {
        const sim::ObjectConstPtr& obj = *it;

        ed::EntityPtr e(new ed::Entity(obj->id()));
        e->setPose(obj->pose());
        e->setShape(obj->shape());

        req.setEntity(e);
    }
}

// ----------------------------------------------------------------------------------------------------

bool SimulatorPlugin::srvSetEntity(ed::SetEntity::Request& req, ed::SetEntity::Response& res)
{
    if (req.action == ed::SetEntity::Request::ADD)
    {
        ed::models::Loader l;
        geo::ShapePtr shape = l.loadShape(req.type);
        if (shape)
        {
            ed::EntityPtr e(new ed::Entity(req.id, req.type));
            e->setShape(shape);

            // Set the pose
            geo::Pose3D pose;
            geo::convert(req.pose, pose);
            e->setPose(pose);

            update_req_->setEntity(e);
        }
        else
        {
            res.error_msg = "No shape could be loaded for type '" + req.type + "'.";
        }
    }
    else if (req.action == ed::SetEntity::Request::DELETE)
    {
        if (world_model_->getEntity(req.id))
        {
            update_req_->removeEntity(req.id);
        }
        else
        {
            res.error_msg = "Entity '" + req.id + "' does not exist.";
        }
    }
    else if (req.action == ed::SetEntity::Request::UPDATE_POSE)
    {
        ed::EntityConstPtr e = world_model_->getEntity(req.id);
        if (e)
        {
            geo::Pose3D new_pose;
            geo::convert(req.pose, new_pose);

            ed::EntityPtr e_new(new ed::Entity(*e));
            e_new->setPose(new_pose);

            update_req_->setEntity(e_new);
        }
        else
        {
            res.error_msg = "Entity '" + req.id + "' does not exist.";
        }
    }

    return true;
}

ED_REGISTER_PLUGIN(SimulatorPlugin)
