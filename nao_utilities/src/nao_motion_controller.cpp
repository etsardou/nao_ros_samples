/**
Copyright (c) 2014, Manos Tsardoulias
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MANOS TSARDOULIAS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "nao_utilities/nao_motion_controller.h"

namespace ros_nao_utils
{
  //! @brief 
  MotionController::MotionController(void)
  {
    
    ros::NodeHandle nh_;
    
    ROS_INFO("Initializing the joint_angles action client");
    angles_client_ = new JointAnglesActionClient("joint_angles_action", true);
    angles_client_->waitForServer();
    ROS_INFO("Server found! joint_angles action client initialized");
    
    ROS_INFO("Initializing the body_pose action client");
    body_pose_client_ = new BodyPoseActionClient("predefined_body_pose", true);
    body_pose_client_->waitForServer();
    ROS_INFO("Server found! body_pose action client initialized");
    
    stiffness_on_service_client = nh_.serviceClient
      <std_srvs::Empty>("body_stiffness/enable");
    stiffness_off_service_client = nh_.serviceClient
      <std_srvs::Empty>("body_stiffness/disable");
  }
  
  //! @brief Executes a movement
  void MotionController::make_movement(
    std::vector<std::string> joints,
    std::vector<float> angles,
    float speed,
    bool relative
  )
  {
    JointAnglesActionGoal g;
    g.joint_angles.joint_names = joints;
    g.joint_angles.joint_angles = angles;
    g.joint_angles.speed = speed;
    g.joint_angles.relative = relative;
    
    angles_client_->sendGoal(g);
    
    bool success = angles_client_->waitForResult(ros::Duration(30.0));
  }
  
  void MotionController::setStiffness(bool state)
  {
    std_srvs::Empty srv;
    if(state)
    {
      if(!stiffness_on_service_client.call(srv))
      {
        ROS_ERROR("'Stifness on' service could not be called"); 
      }
    }
    else
    {
      if(!stiffness_off_service_client.call(srv))
      {
        ROS_ERROR("'Stifness on' service could not be called"); 
      }
    }
  }
  
  void MotionController::setPose(NAOPOSE pose, float speed)
  {
    BodyPoseActionGoal g;
    std::string pose_name;
    switch(pose)
    {
      case STAND:
      {
        pose_name = "Stand";
        break;
      }
      case STANDINIT:
      {
        pose_name = "StandInit";
        break;
      }
      case STANDZERO:
      {
        pose_name = "StandZero";
        break;
      }
      case CROUCH:
      {
        pose_name = "Crouch";
        break;
      }
      case SIT:
      {
        pose_name = "Sit";
        break;
      }
      case SITRELAX:
      {
        pose_name = "SitRelax";
        break;
      }
      case LYINGBELLY:
      {
        pose_name = "LyingBelly";
        break;
      }
      case LYINGBACK:
      {
        pose_name = "LyingBack";
        break;
      }
      default:
      {
        ROS_ERROR("Unknown posture to set");
        return;
      }
    }
    g.posture_name = pose_name;
    g.speed = speed;
    body_pose_client_->sendGoal(g);
    
    bool success = body_pose_client_->waitForResult(ros::Duration(30.0));
  }
  
}
