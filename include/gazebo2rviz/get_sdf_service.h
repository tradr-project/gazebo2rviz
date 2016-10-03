#ifndef GET_SDF_SERVICE_PLUGIN_H
#define GET_SDF_SERVICE_PLUGIN_H

#include <ros/ros.h>
#include <gazebo2rviz/GetSdf.h>
#include <gazebo/gazebo.hh>

#include <map>
#include <string>

namespace gazebo {

/// \brief Gazebo system plugin that allows to get the SDF of models in the simulation through a ROS service call.
///
/// \author Martin Pecka
///
/// Contact: martin.pecka@cvut.cz
///
/// To load this plugin, start gzserver or gazebo with the parameter
/// \code -s /absolute/path/to/libget_sdf_service.so\endcode
class GetSdfServiceSystemPlugin : public SystemPlugin {

public:
    /// \brief The constructor does nothing.
    GetSdfServiceSystemPlugin();
    /// \brief The destructor does nothing.
    ~GetSdfServiceSystemPlugin();

    /// \brief Load the plugin in the simulator and initialize the ROS service.
    /// \param _argc Number of command line arguments.
    /// \param _argv Array of command line arguments.
    void Load(int _argc = 0, char **_argv = NULL);

    /// \brief Reaction of the plugin to the reset signal from the simulation.
    void Reset();

protected:
    /// \brief Try to load the SDF representation of the given model into the internal cache.
    /// \param modelName Name of the model to be loaded.
    /// \return True if the model has been loaded, false on error.
    bool tryLoadSDFIntoCache(std::string const & modelName);

    /// \brief The Gazebo node pointer.
    gazebo::transport::NodePtr gazeboNode;
    /// \brief The callback called when any model gets added/changed (we need to update the cache).
    /// \param msg The serialized model.
    void modelInfoCallback(gazebo::msgs::Model &msg);

    /// \brief Connection to the WorldUpdated event; only used to catch the first update event.
    gazebo::event::ConnectionPtr worldUpdatedEventConnection;
    /// \brief Called when the world is first updated - we set up Gazebo-related things here.
    /// \param updateInfo The update information, holding e.g. the name of the world (which we need).
    void onWorldFirstUpdated(const gazebo::common::UpdateInfo& updateInfo);

    /// \brief The ROS node pointer.
    ros::NodeHandlePtr rosNode;
    /// \brief The server for the get_sdf service.
    ros::ServiceServer getSdfServer;
    /// \brief The callback called when the get_sdf service receives a request.
    /// \param req The request containing the model name.
    /// \param response The response containing the model's SDF.
    /// \return True if the service has succeeded and an SDF is filled to the response; false otherwise.
    ///
    /// In this callback, we read the model's SDF if it is not cached, and return it in the response.
    bool getSdfCallback(gazebo2rviz::GetSdf::Request& req, gazebo2rviz::GetSdf::Response& response);

    /// \brief The internal cache holding the name-SDF mapping.
    std::map<std::string, std::string> modelNamesToXml;

    /// \brief A cached pointer to the world.
    gazebo::physics::WorldPtr world = NULL;
};

}

#endif