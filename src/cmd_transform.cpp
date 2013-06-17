#include "ros/ros.h"
#include "robair_demo/Command.h"
#include <geometry_msgs/Twist.h>
#include <math.h> 
#include <stdint.h>

#define COEF_TURN 100   //==28 (unité moteur)/1(rad/s)
#define COEF_SPEED 160 // ==48(unités moteur)/0.3(m/s)


uint8_t speed1 = 128;
uint8_t turn = 0;

ros::Publisher cmd_pub;

void calculCommand(const geometry_msgs::Twist::ConstPtr& msg)
{
   //transforme la commande de AMCL en speed/turn pour le moteur
   ROS_INFO("I heard: [%lf, %lf]", msg->linear.x, msg->angular.z);

   double vx = msg->linear.x;

   double vt = msg->angular.z;  //max 1 rad/sec

   speed1 = 128 - COEF_SPEED * vx ;
   turn =  128 - COEF_TURN * vt;


   ROS_INFO("speed1 = %d, turn = %d", speed1, turn);

   robair_demo::Command cmd;
   cmd.move = 5;
   cmd.speed1= speed1;
   cmd.turn = turn;
   cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "cmd_transform");

   ros::NodeHandle n;

   // a publisher of /cmd
   cmd_pub = n.advertise<robair_demo::Command>("cmd", 50);
   // subscribing to /cmd_vel
   ros::Subscriber sub = n.subscribe("cmd_vel", 1000, calculCommand);

   ros::spin();

   return 0;
}
