/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

#include <thread>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/OccupancyGrid.h>

#include <global_planner/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include "plan_utils.h"

namespace gazebo
{
  class GZ_PLUGIN_VISIBLE MapActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
  public:
    MapActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
  public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
  public:
    virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
  private:
    void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
  private:
    void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
  private:
    void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Pointer to the parent actor.
  private:
    physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
  private:
    physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
  private:
    sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
  private:
    ignition::math::Vector3d velocity;

    /// \brief List of connections
  private:
    std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
  private:
    ignition::math::Vector3d target;

    /// \brief Target location weight (used for vector field)
  private:
    double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
  private:
    double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
  private:
    double animationFactor = 1.0;

    /// \brief Time of the last update.
  private:
    common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
  private:
    std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
  private:
    physics::TrajectoryInfoPtr trajectoryInfo;

    // ************** Members for Map Based Actor **************
    /// \brief A node use for ROS transport
  private:
    std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
  private:
    ros::Subscriber rosSub;

    /// \brief A ROS publisher
  public:
    ros::Publisher rosPub;

    /// \brief Map of the environment
  public:
    nav_msgs::OccupancyGrid map;

    /// \brief WHether the map arrived so far
  public:
    bool map_arrived = false;

    /// \brief A ROS callbackqueue that helps process messages
  private:
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
  private:
    std::thread rosQueueThread;

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
  public:
    void OnRosMsg(const nav_msgs::OccupancyGridConstPtr &_msg);

    /// \brief ROS helper function that processes messages
  private:
    void QueueThread();
  };
}
#endif
