#include "multi_object_tracker/ros1_pub.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros1_pub");  //初始化

    fusion fs;
    fs.run();
    ros::spin();
    return 0;
}