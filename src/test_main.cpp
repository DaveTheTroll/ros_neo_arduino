#include "ros/ros.h"
#include "ros_neo_arduino/NeoLeds.h"

int main(int argc, char **argv)
{
    const int num_leds = 20;

    ros::init(argc, argv, "coral");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<ros_neo_arduino::NeoLeds>("neo_leds", 20);

    ros_neo_arduino::NeoLeds black;
    ros_neo_arduino::NeoLeds color[6];
    color[0].r = 255;
    color[1].g = 255;
    color[2].b = 255;
    color[3].r = 255; color[3].g = 255;
    color[4].r = 255; color[4].b = 255;
    color[5].g = 255; color[5].b = 255;

    int i=0;
    int j=0;
    ros::Rate rate(10);
    while(ros::ok())
    {
        //black.index = (i + num_leds - 1) % num_leds;
        //pub.publish(black);

        ros_neo_arduino::NeoLeds &msg = color[j];
        msg.index = i;
        pub.publish(msg);
    
        i++;
        if (i == num_leds)
        {
            i = 0;
            j = (j+1)%6;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}