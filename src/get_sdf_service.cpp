#include "gazebo2rviz/get_sdf_service.h"

#include <boost/bind.hpp>

#include <gazebo/physics/physics.hh>

namespace gazebo {

GetSdfServiceSystemPlugin::GetSdfServiceSystemPlugin()
{
}

GetSdfServiceSystemPlugin::~GetSdfServiceSystemPlugin()
{
}

void GetSdfServiceSystemPlugin::Load(int _argc, char **_argv) {
    if (!ros::isInitialized()) {
        ROS_ERROR("Gazebo system plugin gazebo2rviz needs the ros_api_plugin from gazebo_ros_pkgs to be loaded first. "
                  "Not loading the plugin.");
        return;
    }

    // initialize the ROS node and service server
    this->rosNode.reset(new ros::NodeHandle("gazebo2rviz"));
    this->getSdfServer = this->rosNode->advertiseService("get_sdf",
                                                         &GetSdfServiceSystemPlugin::getSdfCallback, this);

    // hook to the first world update event, so that we can find out the name of the world and set up Gazebo subscriber
    this->worldUpdatedEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GetSdfServiceSystemPlugin::onWorldFirstUpdated, this, _1));

    ROS_INFO("gazebo2rviz system plugin loaded.");
}

void GetSdfServiceSystemPlugin::Reset()
{
    this->modelNamesToXml.clear();
}

void GetSdfServiceSystemPlugin::onWorldFirstUpdated(const gazebo::common::UpdateInfo& updateInfo) {
    // make sure the init code is only called once
    gazebo::event::Events::DisconnectWorldUpdateBegin(this->worldUpdatedEventConnection);

    this->world = gazebo::physics::get_world(updateInfo.worldName);
    if (!this->world)
    {
        ROS_FATAL("Cannot load gazebo2rviz plugin, physics::get_world() fails to return world");
        return;
    }

    // initialize the Gazebo node
    this->gazeboNode.reset(new gazebo::transport::Node());
    this->gazeboNode->Init(this->world->GetName());

    ROS_DEBUG("The world has been loaded into the gazebo2rviz system plugin.");
}

bool GetSdfServiceSystemPlugin::tryLoadSDFIntoCache(std::string const & modelName)
{
    if (this->world == NULL) {
        ROS_ERROR("Cannot load model SDF before the simulation has started.");
        return false;
    }

    if (this->world->GetModel(modelName) == NULL)
    {
        return false;
    }

    this->modelNamesToXml[modelName] = this->world->GetModel(modelName)->GetSDF()->ToString("");
    return true;
}

void GetSdfServiceSystemPlugin::modelInfoCallback(gazebo::msgs::Model &msg)
{
    // model was added/updated
    bool success = this->tryLoadSDFIntoCache(msg.name());
    if (!success) {
        ROS_ERROR_STREAM("Could not load the SDF of model " << msg.name().c_str());
    }
}

bool GetSdfServiceSystemPlugin::getSdfCallback(gazebo2rviz::GetSdf::Request& req,
                                               gazebo2rviz::GetSdf::Response& response)
{
    // first, check the cache
    if (this->modelNamesToXml.find(req.model_name) == this->modelNamesToXml.end()) {
        bool success = this->tryLoadSDFIntoCache(req.model_name);
        if (!success) {
            ROS_ERROR_STREAM("Could not load the SDF of model " << req.model_name.c_str());
            return false;
        }
    }

    // return the cached result
    response.sdf = this->modelNamesToXml[req.model_name];
    return true;
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GetSdfServiceSystemPlugin)

}