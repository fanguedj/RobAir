#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h> 
#include <stdint.h>



ros::Publisher pub;

void calculGoal(const geometry_msgs::Pose::ConstPtr& msg)
{
   //transforme la commande de AMCL en speed/turn pour le moteur
   ROS_INFO("I heard: [%lf, %lf]", msg->position.x, msg->position.y);

   geometry_msgs::PoseStamped p;
   p.pose = *msg;
   p.header.stamp = ros::Time::now();
   p.header.frame_id = "map";

   pub.publish(p);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "tabletGoalToNavi");

   ros::NodeHandle n;

   // a publisher of /cmd
   pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 50);
   // subscribing to /cmd_vel
   ros::Subscriber sub = n.subscribe("goal_from_tablet", 1000, calculGoal);

   ros::spin();

   return 0;
}
