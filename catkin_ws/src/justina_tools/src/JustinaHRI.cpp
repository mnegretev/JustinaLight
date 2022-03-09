#include "justina_tools/JustinaHRI.h"

bool JustinaHRI::is_node_set = false;
ros::Subscriber JustinaHRI::subRecognized;
ros::Publisher  JustinaHRI::pubSay;
//Variables for speech
std::string JustinaHRI::_lastRecoSpeech = "";
std::vector<std::string> JustinaHRI::_lastSprHypothesis;
std::vector<float> JustinaHRI::_lastSprConfidences;
bool JustinaHRI::newSprRecognizedReceived = false;

//
//The startSomething functions return inmediately after starting the requested action
//The others, block until the action is finished
//
bool JustinaHRI::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaHRI::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "JustinaHRI.->Setting ros node..." << std::endl;
    subRecognized = nh->subscribe("/hri/sp_rec/recognized", 1, &JustinaHRI::callbackSprRecognized);
    pubSay        = nh->advertise<sound_play::SoundRequest>("/hri/sp_gen/say", 10);
    return true;
}

void JustinaHRI::initRecordAudio(){
    std::cout << "JustinaHRI.->Start to Recording audio" << std::endl;
    std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;
}

void JustinaHRI::stopRecordAudio(std::string test_name, int numQ){
    std::cout << "JustinaHRI.->Stop to Recording audio" << std::endl;
    std::stringstream ss;
    ss.str("");
    ss.clear();
    ss << test_name << "/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/stop_arecord.sh " << numQ;
    std::cout << system(ss.str().c_str()) << std::endl;
}

void JustinaHRI::setInputDevice(DEVICE device){
    std::cout << "JustinaHRI.->Try enable device" << std::endl;
    std::cout << "JustinaHRI.-> ";
    std::stringstream ss;
    ss << pathDeviceScript << "/src/ChangeSourceDevice.sh ";
    switch(device){
        case DEFUALT:
            ss << "-d -e";
            break;
        case KINECT:
            ss << "-k -e";
            break;
        case USB:
            ss << "-u -e";
            break;
        case RODE:
            ss << "-r -e";
            break; 
        default:
            std::cout << "Not device available" << std::endl;
    } 
    std::cout << system(ss.str().c_str()) << std::endl;
}


void JustinaHRI::setOutputDevice(DEVICE device){
    std::cout << "JustinaHRI.->Try enable device" << std::endl;
    std::cout << "JustinaHRI.-> ";
    std::stringstream ss;
    ss << pathDeviceScript << "/src/ChangeSourceDevice.sh ";
    switch(device){
        case DEFUALT:
            ss << "-od -e";
            break;
        case USB:
            ss << "-ou -e";
            break;
        default:
            std::cout << "Not device available" << std::endl;
    } 
    std::cout << system(ss.str().c_str()) << std::endl;
} 

void JustinaHRI::setVolumenInputDevice(DEVICE device, int volumen){
    std::cout << "JustinaHRI.->Try Change the volumen" << std::endl;
    std::cout << "JustinaHRI.-> ";
    std::stringstream ss;
    ss << pathDeviceScript << "/src/ChangeSourceDevice.sh ";
    switch(device){
        case DEFUALT:
            ss << "-d -v " << volumen;
            break;
        case KINECT:
            ss << "-k -v " << volumen;
            break;
        case USB:
            ss << "-u -v " << volumen;
            break;
        default:
            std::cout << "Not device available" << std::endl;
    } 
    std::cout << system(ss.str().c_str()) << std::endl;
} 

void JustinaHRI::setVolumenOutputDevice(DEVICE device, int volumen){
    std::cout << "JustinaHRI.->Try Change the volumen" << std::endl;
    std::cout << "JustinaHRI.-> ";
    std::stringstream ss;
    ss << pathDeviceScript << "/src/ChangeSourceDevice.sh ";
    switch(device){
        case DEFUALT:
            ss << "-od -v " << volumen;
            break;
        case USB:
            ss << "-ou -v " << volumen;
            break;
        default:
            std::cout << "Not device available" << std::endl;
    } 
    std::cout << system(ss.str().c_str()) << std::endl;
}

void JustinaHRI::loadGrammarSpeechRecognized(std::string grammar){
    std::cout << "JustinaHRI.->Load grammar SPR: " << grammar << std::endl;
    if(!usePocketSphinx){
        bbros_bridge::Default_ROS_BB_Bridge srv;
        srv.request.parameters = grammar;
        srv.request.timeout = 10000;
        cltSprGrammar.call(srv);
    }
}

void JustinaHRI::loadGrammarSpeechRecognized(std::string id, std::string grammar){
    hri_msgs::SphinxSetFile msg;
    msg.id = id;
    msg.file_path = grammar;
    pubLoadGrammarPocketSphinx.publish(msg);
}

void JustinaHRI::enableSpeechRecognized(bool enable){

    std::cout << "JustinaHRI.->Enable grammar: " << enable << std::endl;
    if(!usePocketSphinx){
        bbros_bridge::Default_ROS_BB_Bridge srv;
        if(enable)
            srv.request.parameters = "enable";
        else
            srv.request.parameters = "disable";
        srv.request.timeout = 10000;
        cltSprStatus.call(srv);
    }
    else{
        knowledge_msgs::sphinxConf sp_srv;
        if(enable)
            sp_srv.request.flag = true;
        else
            sp_srv.request.flag = false;
        //std_msgs::Bool msg;
        //msg.data = enable;
        //pubEnableSpeechPocketSphinx.publish(msg);
        srvEnableSphinx.call(sp_srv);
    }
}
    
void JustinaHRI::enableGrammarSpeechRecognized(std::string id, float recognitionTime){
    hri_msgs::SphinxSetSearch msg;
    msg.search_id = id;
    msg.recognitionTime = recognitionTime;
    pubEnableGrammarPocketSphinx.publish(msg);
}

//Methos for speech synthesis and recognition
bool JustinaHRI::waitForSpeechRecognized(std::string& recognizedSentence, int timeOut_ms)
{
    newSprRecognizedReceived = false;
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    while(ros::ok() && !newSprRecognizedReceived && --attempts > 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    if(newSprRecognizedReceived)
    {
        recognizedSentence = _lastRecoSpeech;
        return true;
    }
    else
    {
        recognizedSentence = "";
        return false;
    }
}

bool JustinaHRI::waitForSpeechHypothesis(std::vector<std::string>& sentences, std::vector<float>& confidences, int timeOut_ms)
{
    newSprRecognizedReceived = false;
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    while(ros::ok() && !newSprRecognizedReceived && --attempts > 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    if(newSprRecognizedReceived)
    {
        sentences = _lastSprHypothesis;
        confidences = _lastSprConfidences;
        return true;
    }
    else
    {
        sentences.clear();
        confidences.clear();
        return false;
    }
}

bool JustinaHRI::waitForSpecificSentence(std::string expectedSentence, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        if(expectedSentence.compare(sentences[i]) == 0)
            return true;
    return false;
}

bool JustinaHRI::waitForSpecificSentence(std::string option1, std::string option2, std::string& recog, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        if(option1.compare(sentences[i]) == 0 || option2.compare(sentences[i]) == 0)
        {
            recog = sentences[i];
            return true;
        }
    return false;
}

bool JustinaHRI::waitForSpecificSentence(std::string option1, std::string option2, std::string option3,
        std::string& recog, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        if(option1.compare(sentences[i]) == 0 || option2.compare(sentences[i]) == 0 || option3.compare(sentences[i]) == 0)
        {
            recog = sentences[i];
            return true;
        }
    return false;
}

bool JustinaHRI::waitForSpecificSentence(std::string option1, std::string option2, std::string option3, std::string option4,
        std::string& recog, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        if(option1.compare(sentences[i]) == 0 || option2.compare(sentences[i]) == 0 ||
                option3.compare(sentences[i]) == 0 || option4.compare(sentences[i]) == 0)
        {
            recog = sentences[i];
            return true;
        }
    return false;
}

bool JustinaHRI::waitForSpecificSentence(std::vector<std::string>& options, std::string& recognized, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        for(size_t j=0; j<options.size(); j++)
            if(options[j].compare(sentences[i]) == 0)
            {
                recognized = sentences[i];
                return true;
            }
    return false;
}

bool JustinaHRI::waitForUserConfirmation(bool& confirmation, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
    {
        if(sentences[i].compare("robot yes") == 0 || sentences[i].compare("justina yes") == 0)
        {
            confirmation = true;
            return true;
        }
        if(sentences[i].compare("robot no") == 0 || sentences[i].compare("justina no") == 0)
        {
            confirmation = false;
            return true;
        }
    }
    return false;
}

std::string JustinaHRI::lastRecogSpeech()
{
    return _lastRecoSpeech;
}

void JustinaHRI::fakeSpeechRecognized(std::string sentence)
{
    std_msgs::String str;
    hri_msgs::RecognizedSpeech spr;
    str.data = sentence;
    spr.hypothesis.push_back(sentence);
    spr.confidences.push_back(0.9);
    pubFakeSprRecognized.publish(str);
    pubFakeSprHypothesis.publish(spr);
}

void JustinaHRI::startSay(std::string strToSay)
{
}

void JustinaHRI::say(std::string strToSay)
{
    std::cout << "JustinaHRI.->Saying: " << strToSay << std::endl;
    bbros_bridge::Default_ROS_BB_Bridge srv;
    srv.request.parameters = strToSay;
    srv.request.timeout = 10000;
    cltSpgSay.call(srv);
}


void JustinaHRI::startHybridFollow()
{
    
    std_srvs::Trigger srv;
    if (cltSRoiTrack.call(srv)){
        std::cout << "TRUE ROI TRACK" << std::endl;
        std_msgs::Bool msg;
        msg.data = true;
        JustinaHRI::pubHybridFollow.publish(msg);
    }
    /*else{
        std::cout << "FALSE ROI TRACK" << std::endl;
        std_msgs::Bool msg;
        msg.data = false;
        JustinaHRI::pubHybridFollow.publish(msg);
    }*/
}

void JustinaHRI::stopHybridFollow()
{
    std_srvs::Empty srv;
    cltstopRoiTrack.call(srv);
    
    std::cout << "SHUTDOWN ROI TRACKER" << std::endl;
    std_msgs::Bool msg;
    msg.data = false;
    JustinaHRI::pubHybridFollow.publish(msg);
    
}


//Methods for human following
void JustinaHRI::startFollowHuman()
{
    std_msgs::Bool msg;
    msg.data = true;
    JustinaHRI::pubFollowStartStop.publish(msg);
}

void JustinaHRI::stopFollowHuman()
{
    std_msgs::Bool msg;
    msg.data = false;
    JustinaHRI::pubFollowStartStop.publish(msg);
}

void JustinaHRI::enableLegFinder(bool enable)
{
    if(!enable)
    {
        JustinaHRI::_legsFound = false;
        std::cout << "JustinaHRI.->Leg_finder disabled. " << std::endl;
    }
    else
        std::cout << "JustinaHRI.->Leg_finder enabled." << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    JustinaHRI::pubLegsEnable.publish(msg);
}

void JustinaHRI::enableLegFinderRear(bool enable)
{
    if(!enable)
    {

        std::cout << "JustinaHRI.->Leg_finder_rear disabled. " << std::endl;
    }
    else
        std::cout << "JustinaHRI.->Leg_finder_rear enabled." << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    JustinaHRI::pubLegsRearEnable.publish(msg);
}

bool JustinaHRI::frontalLegsFound()
{
    return JustinaHRI::_legsFound;
}

bool JustinaHRI::rearLegsFound()
{
    return JustinaHRI::_legsRearFound;
}

void JustinaHRI::getLatestLegsPoses(float &x, float &y)
{
    x = JustinaHRI::lastLegsPoses.point.x;
    y = JustinaHRI::lastLegsPoses.point.y;
}

void JustinaHRI::getLatestLegsPosesRear(float &x, float &y)
{
    x = JustinaHRI::lastLegsPosesRear.point.x;
    y = JustinaHRI::lastLegsPosesRear.point.y;
}

void JustinaHRI::callbackSprRecognized(const std_msgs::String::ConstPtr& msg)
{
    _lastRecoSpeech = msg->data;
    newSprRecognizedReceived = true;
    std::cout << "JustinaHRI.->Received recognized speech: " << msg->data << std::endl;
}

void JustinaHRI::callbackSprHypothesis(const hri_msgs::RecognizedSpeech::ConstPtr& msg)
{
    if(msg->hypothesis.size() < 1 || msg->confidences.size() < 1)
    {
        std::cout << "JustinaHRI.->Invalid speech recog hypothesis: msg is empty" << std::endl;
        return;
    }
    _lastRecoSpeech = msg->hypothesis[0];
    _lastSprHypothesis = msg->hypothesis;
    _lastSprConfidences = msg->confidences;
    std::cout << "JustinaHRI.->Last reco speech: " << _lastRecoSpeech << std::endl;
    newSprRecognizedReceived = true;
}

void JustinaHRI::callbackLegsFound(const std_msgs::Bool::ConstPtr& msg)
{
    // std::cout << "JustinaHRI.->Legs found signal received!" << std::endl;
    JustinaHRI::_legsFound = msg->data;
}

void JustinaHRI::callbackLegsRearFound(const std_msgs::Bool::ConstPtr& msg)
{
    //std::cout << "JustinaHRI.->Legs rear found signal received!" << std::endl;
    JustinaHRI::_legsRearFound = msg->data;
}

void JustinaHRI::callbackLegsPoses(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    JustinaHRI::lastLegsPoses = *msg;
}

void JustinaHRI::callbackLegsPosesRear(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    JustinaHRI::lastLegsPosesRear = *msg;
}

//Methods for qr reader
void JustinaHRI::callbackQRRecognized(const std_msgs::String::ConstPtr& msg){
    std::cout << "JustinaHRI.->Qr reader received" << std::endl;
    boost::posix_time::ptime timeCurrQRReceived = boost::posix_time::second_clock::local_time();
    if(lastQRReceived.compare(msg->data) != 0 || (timeCurrQRReceived - timeLastQRReceived).total_milliseconds() > 5000){
        timeLastQRReceived = boost::posix_time::second_clock::local_time();
        lastQRReceived = msg->data;
        std_msgs::String str;
        hri_msgs::RecognizedSpeech spr;
        str.data = msg->data;
        spr.hypothesis.push_back(msg->data);
        spr.confidences.push_back(0.9);
        pubFakeSprRecognized.publish(str);
        pubFakeSprHypothesis.publish(spr);
    }
}

void JustinaHRI::callbackBusy(const std_msgs::String::ConstPtr& msg){
		
	//std::cout  << "--------- Busy Callback ---------" << std::endl;
    if(msg->data == "start_spgen"){
        spgenbusy = true;
    }
    else if(msg->data == "finish_spgen"){
        spgenbusy = false;
	    JustinaHRI::asyncSpeech();
    }


	//std::cout << "SPGEN Status:" << msg->data << std::endl;
	
	//JustinaHRI::asyncSpeech();
}

bool JustinaHRI::waitAfterSay(std::string strToSay, int timeout, int delay){
    bbros_bridge::Default_ROS_BB_Bridge srv;
    srv.request.parameters = strToSay;
    srv.request.timeout = timeout;
    if (cltSpgSay.call(srv)) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(delay));
        return true;
    }
    return false;
}

void JustinaHRI::playSound()
{
    std::cout << "JudtinsHRI.->Playing sound!" << std::endl;
    sound_play::Sound sound = sc->waveSoundFromPkg("knowledge", "sounds/R2D2a.wav");
    sound.play();
    sleep(3.0);
    sound.stop();
}

/*void JustinaHRI::initRoiTracker(){
    std::cout << "JustinaHRI.->Init Roi Tracker" << std::endl;
    std_srvs::Trigger srv;
    if (cltSRoiTrack.call(srv)){
		std::cout << "TRUE ROI TRACK" << std::endl;
	}
	else
		std::cout << "FALSE ROI TRACK" << std::endl;
}*/

/*void JustinaHRI::stopRoiTracker()
{
    std::cout << "JustinaHRI.->Stop Roi Tracker" << std::endl;
    std_srvs::Trigger srv;
    if (cltstopRoiTrack.call(srv)){
        std::cout << "TRUE ROI TRACK" << std::endl;
    }
    else
        std::cout << "FALSE ROI TRACK" << std::endl;
}*/

int JustinaHRI::inicializa(){
	if((tas = (JustinaHRI::Queue*)malloc(sizeof(JustinaHRI::Queue)))==NULL)
        	return -1;
	tas->inicio = NULL;
	tas->ultimo = NULL;
	tas->tam = 0;
	return 0;
}

int JustinaHRI::insertAsyncSpeech(std::string dato, int time, int ros_time, int limit_time){
	elemento *newelemento;

	if((newelemento=(elemento*)malloc(sizeof(elemento))) == NULL)
		return -1;
	if((newelemento->time=(int*)malloc(sizeof(int))) == NULL)
		return -1;
	if((newelemento->ros_time=(int*)malloc(sizeof(int))) == NULL)
		return -1;
	if((newelemento->limit_time=(int*)malloc(sizeof(int))) == NULL)
		return -1;

	newelemento->dato = new std::string(dato);
	newelemento->time[0] = time;
    newelemento->ros_time[0] = ros_time;
    newelemento->limit_time[0] = limit_time;
	newelemento->siguiente = NULL;
	if(tas->ultimo !=NULL)
		tas->ultimo->siguiente = newelemento;
	tas->ultimo = newelemento;
	if(tas->inicio == NULL)
		tas->inicio = newelemento;
	tas->tam++;
    if(!spgenbusy){
        std_msgs::String msg;
        msg.data = "finish_spgen";
        pubSpGenBusy.publish(msg);
    }
	return 0;
}

int JustinaHRI::asyncSpeech(){
	elemento *sup_elemento;
	if (tas->tam == 0)
		return -1;

    ros::Time time;
	sup_elemento = tas->inicio;
	tas->inicio = tas->inicio->siguiente;

    time = ros::Time::now();
    //std::cout << "Actual Time: " << time.sec << " Limit time: " << sup_elemento->ros_time[0] + sup_elemento->limit_time[0] << std::endl; 
    if(time.sec < sup_elemento->ros_time[0] + sup_elemento->limit_time[0]){
    	bbros_bridge::Default_ROS_BB_Bridge srv;
	srv.request.parameters = sup_elemento->dato[0];
        boost::this_thread::sleep(boost::posix_time::milliseconds(sup_elemento->time[0]));
	srv.request.timeout = 10000;
	cltSpgSay.call(srv);
    }
    else{
        //std::cout << "Out of Time, Not speech: " << sup_elemento->dato[0] << std::endl;
        std_msgs::String msg;
        msg.data = "finish_spgen";
        pubSpGenBusy.publish(msg);
    }

	delete sup_elemento->dato;//free(sup_elemento->dato);
	free(sup_elemento);
	if(tas->inicio == NULL)
		tas->ultimo = NULL;
	tas->tam--;
	return 0;
}

void JustinaHRI::view(){
	elemento *actual;
	int i;

	actual = tas->inicio;

	for(i=0; i < tas->tam; i++){
		std::cout << "\t " << actual->dato[0] << std::endl;
		//printf("\t %s \n", actual->dato);
		actual = actual->siguiente;
	}
}
