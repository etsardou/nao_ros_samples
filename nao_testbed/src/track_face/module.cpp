/**
Copyright (c) 2014, Manos Tsardoulias, CERTH/ITI
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the CERTH/ITI nor the
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

#include "nao_testbed/track_face/module.h"

Module::Module(void)
{
  setStiffness(true);
  speak("Stiffness on");
}

void Module::face_detection_callback(const nao_msgs::FaceDetected& msg)
{
  if(face_detection_lock)
  {
    return;
  }
  face_detection_lock = true;
  
  std::vector<std::string> joints;
  std::vector<float> angles;
  float speed = 0.5;
  bool relative = 1;
  
  joints.push_back("HeadYaw");
  joints.push_back("HeadPitch");
  angles.push_back(msg.face_info.shape_info.alpha.data * 0.4);
  angles.push_back(msg.face_info.shape_info.beta.data * 0.4);
  
  make_movement(joints, angles, speed, relative);
  
  face_detection_lock = false;
}

void Module::tactile_callback(const nao_msgs::TactileTouch& msg)
{
  if(msg.state == 0)
  {
    return;
  }
  if(msg.button == 2)
  {
    setStiffness(false);
    speak("Stiffness off");
  }
}

