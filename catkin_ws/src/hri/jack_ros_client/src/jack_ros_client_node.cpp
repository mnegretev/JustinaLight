#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "jack/jack.h"

#define SAMPLING_RATE 48000
#define NFRAMES 1024

jack_port_t* input_port;
bool is_jack_running;
ros::Publisher pub_raw_microphone;
std_msgs::Float32MultiArray msg_raw_microphone;

int jack_callback (jack_nframes_t nframes, void *arg)
{
    jack_default_audio_sample_t* in;    
    in = (jack_default_audio_sample_t*)jack_port_get_buffer (input_port, nframes);
    // float power = 0;
    // for(int i=0; i < nframes; i++)
    //     power += in[i]*in[i];
    // power /= nframes;
    // std::cout << "Power: " << power << std::endl;
    for(size_t i=0; i< nframes; i++)
        msg_raw_microphone.data[i] = in[i];
    pub_raw_microphone.publish(msg_raw_microphone);
    return 0;
}

void jack_shutdown (void *arg)
{
    exit(-1);
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING JACK ROS CLIENT NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "jack_ros_client");
    ros::NodeHandle n;
    pub_raw_microphone = n.advertise<std_msgs::Float32MultiArray>("/hri/microphone_raw", 10);
    msg_raw_microphone.data.resize(NFRAMES);
    ros::Rate loop(100);
    
    std::cout << "JackROSClient.->Trying to connect to jack server... " << std::endl;
    const char* client_name = "jack_ros_client";
    jack_status_t status;
    jack_client_t* client = jack_client_open (client_name, JackNoStartServer, &status);
    if (client == NULL)
    {
        std::cout << "JackROSClient.->jack_client_open() failed with status = " << status << std::endl;
        if (status & JackServerFailed) 
            std::cout << "JackROSClient.->Unable to connect to JACK server." << std::endl;
        return -1;
    }
    if (status & JackNameNotUnique)
    {
        client_name = jack_get_client_name(client);
        std::cout << "JackROSClient.->WARNING:Another agent with same name is running, "<<client_name<<" has been assigned to us."<<std::endl;
    }

    jack_set_process_callback (client, jack_callback, 0);
    jack_on_shutdown (client, jack_shutdown, 0);

    std::cout << "JackROSClient.->Engine sample rate: " <<  jack_get_sample_rate (client) << std::endl;
    std::cout << "JackROSClient.->Trying to create agent ports..." << std::endl;
    input_port = jack_port_register (client, "input", JACK_DEFAULT_AUDIO_TYPE,JackPortIsInput, 0);
    if (input_port == NULL)
    {
        std::cout << "JackROSClient.->Could not create agent ports. Have we reached the maximum amount of JACK agent ports?" << std::endl;
        return -1;
    }
    if (jack_activate(client))
    {
        std::cout << "JackROSClient.->Cannot activate client." << std::endl;
        return -1;
    }

    std::cout << "JackROSClient.->Connecting ports... " << std::endl;
    const char **serverports_names;
    serverports_names = jack_get_ports(client, NULL, NULL, JackPortIsPhysical|JackPortIsOutput);
    if (serverports_names == NULL)
    {
        std::cout << "JackROSClient.->No available physical capture (server output) ports." << std::endl;
        return -1;
    }
    if (jack_connect(client, serverports_names[0], jack_port_name(input_port)))
    {
        std::cout << "JackROSClient.->Cannot connect input port." << std::endl;
        return -1;
    }
    free(serverports_names);
    
    std::cout << "JackROSClient.->I think everything is ok (Y)" << std::endl;
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
	
    jack_client_close (client);
    return 0;
}
