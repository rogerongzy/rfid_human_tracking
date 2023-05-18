#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include"rfid/rfid_msg.h"

void subscriberCallback(const rfid::rfid_msg::ConstPtr& msg)
{
    printf("epc=%s, time=%010llu, idx=%02u, mode=%04u, ant=%01u, ph=%f, rssi=%f\n",
           msg->epc.c_str() ,msg->time, msg->idx, msg->mode, msg->ant, msg->phase, msg->rssi);
}

int main(int argc, char **argv)
{
        ros::Subscriber rfid_subscriber;
        ros::init(argc, argv,"rfid_subscriber");
        ros::NodeHandle nh;
        rfid_subscriber = nh.subscribe("/rfid_message", 15, &subscriberCallback);
        // queue_size should be set so that subscriber is as fast as or faster than publisher, so that no data will be missed

        ////////////////////////////
        ros::Rate loop_rate(100); // also help not losing data
        ////////////////////////////
        
        
        
        while (ros::ok())
        {
               loop_rate.sleep();
                ros::spinOnce(); // ros::spin();
        }

        return 0;
}
