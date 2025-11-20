#include <apriltag_approaching/apriltag_approaching.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "apriltag_approaching_node");

    ros::NodeHandle nh;

    ros::Rate rate(30);

    APRILTAG_APPROACHING tag_approaching(&nh, rate);

    int counter = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        tag_approaching.computeAlignment();
        rate.sleep();
    }
}