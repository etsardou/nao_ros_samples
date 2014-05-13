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

#include "nao_testbed/vocal_commands_posture_change/module.h"

Module::Module(void)
{
  std::vector<std::string> words;
  words.push_back("yes");
  words.push_back("no");
  setVocabulary(words);
  
  startRecognition();
}


void Module::tactile_callback(const nao_msgs::TactileTouch& msg)
{
  if(tactile_lock)
  {
    return;
  }
  if(msg.state == 0)
  {
    return;
  }
  
  tactile_lock = true;
  
  std::string word = "";
  switch(msg.button)
  {
    case 2: //! Middle
      stopRecognition();
      break;
  }

  tactile_lock = false;
}

void Module::wordRecognizedCallback(const nao_msgs::WordRecognized& msg)
{
  stopRecognition();
  
  std::string best_guess = "";
  float best_guess_val = 0;
  
  if(msg.words.size() == 0)
  {
    speak("Did not understand. Please repeat.");
    startRecognition();
    return;
  }
  for(unsigned int i = 0 ; i < msg.words.size() ; i++)
  {
    if(msg.confidence_values[i] > best_guess_val)
    {
      best_guess_val = msg.confidence_values[i];
      best_guess = msg.words[i];
    }
  }
  
  prompt(best_guess);
}
