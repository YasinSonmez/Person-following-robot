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

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "MapActorPlugin.hh"

// using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(gazebo::MapActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
gazebo::MapActorPlugin::MapActorPlugin()
{
}

/////////////////////////////////////////////////
void gazebo::MapActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO("Entry1");
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&gazebo::MapActorPlugin::OnUpdate, this, std::placeholders::_1)));
  ROS_INFO("Entry2");
  this->Reset();
  ROS_INFO("Entry3");
  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  ROS_INFO("Entry4");
  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    ROS_INFO("Entry5");
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
              ros::init_options::NoSigintHandler);
  }
  ROS_INFO("Entry6");
  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
  ROS_INFO("Entry7");
  // Publisher setup
  this->rosPub = this->rosNode->advertise<std_msgs::Int8>("/" + this->actor->GetName() + "/test_publisher", 1);
  ROS_INFO("Entry8");
  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>(
          "/map",
          1,
          boost::bind(&gazebo::MapActorPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  // Costmap conversion
  ROS_INFO("Entry9");
  // tf2_ros::Buffer buffer(ros::Duration(10));
  // tf2_ros::TransformListener tf(buffer);

  tf_buffer_.reset(new tf2_ros::Buffer());
  ROS_INFO("Entry10");
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  ROS_INFO("Entry11");
  costmap_2d::Costmap2DROS lcr("costmap", *tf_buffer_);
  ROS_INFO("Entry12");
  // global_planner::PlannerWithCostmap pppp("planner", &lcr);
  ROS_INFO("Entry13");
  //  std::cout << pppp.costmap_->data[0] << std::endl;

  // Spin up the queue helper thread.
  this->rosQueueThread =
      std::thread(std::bind(&gazebo::MapActorPlugin::QueueThread, this));
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
void gazebo::MapActorPlugin::OnRosMsg(const nav_msgs::OccupancyGridConstPtr &_msg)
{
  map_arrived = true;
  this->map = *_msg;
}

/// \brief ROS helper function that processes messages
void gazebo::MapActorPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
void gazebo::MapActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void gazebo::MapActorPlugin::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos() - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
void gazebo::MapActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                  model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
                                        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void gazebo::MapActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  if (this->map_arrived)
  {
    std_msgs::Int8 map_msg;
    map_msg.data = this->map.data[0];
    this->rosPub.publish(map_msg);
  }

  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  // this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian() * 0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
                             this->actor->WorldPose().Pos())
                                .Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
                             (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
