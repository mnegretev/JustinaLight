#pragma once
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sound_play/RequestSound.h"
#include "geometry_msgs/PointStamped.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/thread/thread.hpp"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
#include "hri_msgs/RecognizedSpeech.h"

class JustinaHRI
{
private:
    static bool is_node_set;
    //Members for operating speech synthesis and recognition. (Assuming that blackboard modules are used)
    static ros::Publisher  pubSay;
    static ros::Subscriber subRecognized;

    //Variables for speech
    static std::string _lastRecoSpeech;
    static std::vector<std::string> _lastSprHypothesis;
    static std::vector<float> _lastSprConfidences;
    static bool newSprRecognizedReceived;


public:
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for speech synthesis and recognition
    static bool waitForSpeechRecognized(std::string& recognizedSentence, int timeOut_ms);
    static bool waitForSpeechHypothesis(std::vector<std::string>& sentences, std::vector<float>& confidences, int timeOut_ms);
    static bool waitForSpecificSentence(std::string expectedSentence, int timeOut_ms);
    static bool waitForSpecificSentence(std::string option1, std::string option2, std::string& recog, int timeOut_ms);
    static bool waitForSpecificSentence(std::string option1, std::string option2, std::string option3,
                                        std::string& recog, int timeOut_ms);
    static bool waitForSpecificSentence(std::vector<std::string>& options, std::string& recognized, int timeOut_ms);
    static bool waitForUserConfirmation(bool& confirmation, int timeOut_ms);
    static std::string lastRecogSpeech();
    static void startSay(std::string strToSay);
    static void say(std::string strToSay, int timeout);
    static void playSound(); 

private:
    //Speech recog and synthesis
    static void callbackSprHypothesis(const hri_msgs::RecognizedSpeech::ConstPtr& msg);
};
