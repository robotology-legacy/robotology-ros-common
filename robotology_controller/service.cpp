#include "ros/ros.h"
#include <robotology_msgs/controlBoardSrv.h>

bool add(robotology_msgs::controlBoardSrv::Request  &req,
         robotology_msgs::controlBoardSrv::Response &res)
{
    res.success = (req.ciao == 42 ? true : false);
    ROS_INFO("request: x=%ld -> response %d\n", (long int)req.ciao, (int) res.success);
//     ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_worlds_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("ciao", add);
    ROS_INFO("Ready to rule the world.");
    ros::spin();

    return 0;
}
