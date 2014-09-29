#include "simulator_plugin.h"

#include "world.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/models/models.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <geolib/ros/msg_conversions.h>

// Simulator
#include "object.h"

// ----------------------------------------------------------------------------------------------------

void addToUpdateRequest(const sim::ObjectConstPtr& obj, ed::UpdateRequest& req)
{
    ed::EntityPtr e(new ed::Entity(obj->id()));
    e->setPose(obj->pose());
    e->setType(obj->type());
    e->setShape(obj->shape());

    req.setEntity(e);
}

// ----------------------------------------------------------------------------------------------------

SimulatorPlugin::SimulatorPlugin() : reconfigured_(false)
{
    std::cout << "INIT" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

SimulatorPlugin::~SimulatorPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void SimulatorPlugin::configure(tue::Configuration config)
{
    simulator_.configure(config);
    reconfigured_ = true;
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

    if (reconfigured_)
    {
        // The simulator was reconfigured, so updated all objects

        // Update ED
        const std::map<sim::UUID, sim::ObjectConstPtr>& sim_objects = simulator_.world()->objects;
        for(std::map<sim::UUID, sim::ObjectConstPtr>::const_iterator it = sim_objects.begin(); it != sim_objects.end(); ++it)
        {
            addToUpdateRequest(it->second, req);
        }

        reconfigured_ = false;
    }
    else
    {
        // Only update all changed objects in the world model
        for(std::vector<sim::ObjectConstPtr>::const_iterator it = changed_objects.begin(); it != changed_objects.end(); ++it)
        {
            addToUpdateRequest(*it, req);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

bool SimulatorPlugin::srvSetEntity(ed::SetEntity::Request& req, ed::SetEntity::Response& res)
{
    if (req.action == ed::SetEntity::Request::ADD)
    {
        ed::models::NewEntityConstPtr e_created = ed::models::create(req.type, tue::Configuration(), req.id);
        if (e_created && e_created->shape)
        {
            ed::EntityPtr e(new ed::Entity(req.id, req.type));
            e->setShape(e_created->shape);

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
