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

#ifndef ROS_NAO_SPEECH_WRAPPER
#define ROS_NAO_SPEECH_WRAPPER

#include "nao_utilities/includes.h"

//! @namespace ros_nao_utils
//! @brief Namespace for the ROS NAO wrappers
namespace ros_nao_utils
{
  
  typedef actionlib::SimpleActionClient<nao_msgs::SetSpeechVocabularyAction> 
    SetSpeechVocabularyActionClient; 
  typedef nao_msgs::SetSpeechVocabularyGoal SetSpeechVocabularyActionGoal;
  typedef const nao_msgs::SetSpeechVocabularyResult::ConstPtr& 
    SetSpeechVocabularyActionResultPtr;
    
  typedef actionlib::SimpleActionClient<nao_msgs::SpeechWithFeedbackAction> 
    SpeechWithFeedbackActionClient; 
  typedef nao_msgs::SpeechWithFeedbackGoal SpeechWithFeedbackActionGoal;
  typedef const nao_msgs::SpeechWithFeedbackResult::ConstPtr& 
    SpeechWithFeedbackActionResultPtr;
  
  //! @class Speech
  //! @brief Wrapper for NAO's speech module 
  class Speech
  {
    private:
    
      ros::Publisher speak_pub_;
      
      SetSpeechVocabularyActionClient *set_voc_act_client_;
      SpeechWithFeedbackActionClient *speech_act_client_;
      
      //!< The dynamic reconfigure parameters' server
      dynamic_reconfigure::Server
        <nao_driver::nao_speechConfig> server;
        
      ros::ServiceClient start_recognition_service_client_;
      ros::ServiceClient stop_recognition_service_client_;
      
      ros::Subscriber word_recognized_sub_;
    
    public:
    
      //! @brief Default constructor 
      Speech(void);
      
      //! @brief Makes NAO say the input sentence
      void speak(std::string s);
      
      void speakWithFeedback(std::string s);
      
      void startRecognition(void);
      void stopRecognition(void);
      
      virtual void wordRecognizedCallback(
        const nao_msgs::WordRecognized& msg);
        
      void setVocabulary(std::vector<std::string> vocabulary);
      
      void prompt(std::string word);
  };
}
//~ 
#endif
