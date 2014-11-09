#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include <ras_srv_msgs/Command.h>
#include "ras_utils/controller.h"
#include "ras_utils/basic_node.h"
#include "ras_utils/node_communication.h"
#include "std_msgs/String.h"

#define PUBLISH_RATE    10      //TODO: Maybe lower?
#define QUEUE_SIZE      1000

/**
 * @brief Controls the whole robot
 */
class Brain : rob::BasicNode
{

public:
    Brain();
    void run();
private:
    // ** Publishers
    ros::Publisher activate_node_pub_;

    // ** Subscribers
    ros::Subscriber adc_sub_;

    // ** Callbacks
    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg);

    // Service callback
    bool srvCallback(ras_srv_msgs::Command::Request &req, ras_srv_msgs::Command::Response &resp);

    double front_right_distance_;
    double back_right_distance_;
    double front_left_distance_;
    double back_left_distance_;
    double front_distance_;
    double back_distance_;

    bool isCloseToWall();

};

int main (int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, NODE_BRAIN);

    // ** Create wall follower object
    Brain brain;

    // ** Run
    brain.run();
    ros::spin();
}

Brain::Brain()
{
    // Publisher
    activate_node_pub_ = n.advertise<std_msgs::String>(ACTIVATE_NODE_TOPIC_NAME, QUEUE_SIZE);

    // Subscriber
    adc_sub_ = n.subscribe(TOPIC_ARDUINO_ADC, QUEUE_SIZE,  &Brain::adcCallback, this);

    // Services
    srv_in_ = n.advertiseService(SRV_BRAIN_IN, &Brain::srvCallback, this);
}

void Brain::run()
{
    ros::Rate loop_rate(PUBLISH_RATE);
    bool close_to_wall;
    int i = 0;
    while(ros::ok())
    {
//        close_to_wall = isCloseToWall();
        if(communicate(SRV_NAVIGATION_IN, (i%2)+1))
            ROS_INFO("BRAIN communicates");
        else
            ROS_ERROR("CANT COMMUNICATE");
        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }
}

bool Brain::isCloseToWall()
{

}

void Brain::adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg)
{
    front_right_distance_ = RAS_Utils::shortSensorToDistanceInCM(msg->ch4);
    back_right_distance_ = RAS_Utils::shortSensorToDistanceInCM(msg->ch3);

    front_left_distance_ = RAS_Utils::shortSensorToDistanceInCM(msg->ch1);
    back_left_distance_ = RAS_Utils::shortSensorToDistanceInCM(msg->ch2);

    // TODO: Not sure about which one ic ch5 or ch6
    // TODO: Will probably need longSensorToDistanceInCM to work as intended
    front_distance_ = RAS_Utils::shortSensorToDistanceInCM(msg->ch5);
    back_distance_ = RAS_Utils::shortSensorToDistanceInCM(msg->ch6);

}

bool Brain::srvCallback(ras_srv_msgs::Command::Request &req, ras_srv_msgs::Command::Response &resp)
{
    ROS_INFO("Brain receives command: %ld", req.command);
    resp.result = 1;
    return true;
}
