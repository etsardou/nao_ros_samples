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

#include "nao_utilities/nao_speech.h"

namespace ros_nao_utils
{
  //! @brief Default constructor
  Speech::Speech(void)
  {
    ros::NodeHandle nh_;
    //~ Subscription to /speech
    speak_pub_ = nh_.advertise<std_msgs::String>("speech",5);
    
    start_recognition_service_client_ = nh_.serviceClient
      <std_srvs::Empty>("start_recognition");

    stop_recognition_service_client_ = nh_.serviceClient
      <std_srvs::Empty>("stop_recognition");
      
    word_recognized_sub_ = nh_.subscribe(
      "word_recognized", 
      1, 
      &Speech::wordRecognizedCallback,
      this);
      
    ROS_INFO("Initializing the set speech vocabulary action client");
    set_voc_act_client_ = 
      new SetSpeechVocabularyActionClient("speech_vocabulary_action", true);
    set_voc_act_client_->waitForServer();
    ROS_INFO("Server found! set speech vocabulary action client initialized");
    
    ROS_INFO("Initializing the speech with feedback action client");
    speech_act_client_ = 
      new SpeechWithFeedbackActionClient("speech_action", true);
    speech_act_client_->waitForServer();
    ROS_INFO("Server found! set speech with feedback action client initialized");
  }
  
  //! @brief Makes NAO say the input sentence
  void Speech::speak(std::string s)
  {
    std_msgs::String msg;
    msg.data = s;
    if(!speak_pub_)
    {
      ROS_ERROR("Speech publisher is not operating");
      return;
    }
    speak_pub_.publish(msg);
  }
  
  //! @brief Makes NAO say the input sentence
  void Speech::speakWithFeedback(std::string s)
  {
    SpeechWithFeedbackActionGoal g;
    g.say = s;
    
    speech_act_client_->sendGoal(g);
    bool success = speech_act_client_->waitForResult(ros::Duration(30.0));
    if(!success)
    {
      ROS_ERROR("SpeechWithFeedbackAction result timed out");
    }
  }
  
  void Speech::startRecognition(void)
  {
    std_srvs::Empty srv;
    if(!start_recognition_service_client_.call(srv))
    {
      ROS_ERROR("Could not start speech recognition"); 
    }
  }
  
  void Speech::stopRecognition(void)
  {
    std_srvs::Empty srv;
    if(!stop_recognition_service_client_.call(srv))
    {
      ROS_ERROR("Could not stop speech recognition"); 
    }
  }
  
  void Speech::wordRecognizedCallback
    (const nao_msgs::WordRecognized& msg){}
    
  void Speech::setVocabulary(std::vector<std::string> vocabulary)
  {
    SetSpeechVocabularyActionGoal g;
    g.words = vocabulary;
    
    set_voc_act_client_->sendGoal(g);
    bool success = set_voc_act_client_->waitForResult(ros::Duration(30.0));
    if(!success)
    {
      ROS_ERROR("SetSpeechVocabularyAction result timed out");
    }
    
    SetSpeechVocabularyActionResultPtr res = set_voc_act_client_->getResult();
    if(!res->success)
    {
      ROS_ERROR("Something went wrong with setting the speech vocabulary.");
      ROS_ERROR("Did you send an empty list?");
    }
  }
  
  void Speech::prompt(std::string word)
  {
    std::vector<std::string> vocabulary;
    vocabulary.push_back("no");
    vocabulary.push_back("yes");
    setVocabulary(vocabulary);
    speakWithFeedback(std::string("Did you say ") + word + std::string(" ?"));
    startRecognition();
  }
}
