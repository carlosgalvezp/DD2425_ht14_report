#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_utils/controller.h"
#include "ras_utils/basic_node.h"

/**
 * @brief Controls the whole robot
 */
class Brain : rob::BasicNode
{
public:
    Brain();
    void run();

private:
    ros::NodeHandle n_;
    // ** Publishers

    // ** Subscribers

    // ** Callbacks
};

int main (int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "brain");

    // ** Create wall follower object
    Brain brain;

    // ** Run
    brain.run();
}

Brain::Brain(){}

void Brain::run()
{

}

