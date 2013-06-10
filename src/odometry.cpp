#include "ros/ros.h"
#include "robair_demo/encoderData.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h> 
#include <stdint.h>

#define TICKS_PAR_TOUR 980//588
#define DIAMETRE_ROUE  0.125 //en m
#define ENTRE_ROUES 0.325 //en m
#define TICK_TO_RAD ((DIAMETRE_ROUE*2*M_PI)/(TICKS_PAR_TOUR*ENTRE_ROUES))
#define TICKS_TO_M ((M_PI*DIAMETRE_ROUE)/TICKS_PAR_TOUR)

int32_t tickG = 0;
int32_t tickD = 0;
int32_t tickGPrec = 0;
int32_t tickDPrec = 0;

double x=0,y=0,th=0;
double vx=0, vy=0, vth=0;

ros::Time currentTime;

int doPublish = false;

void publish(ros::Publisher odom_pub, tf::TransformBroadcaster odom_broadcaster)
{

   //since all odometry is 6DOF we'll need a quaternion created from yaw
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

   //first, we'll publish the transform over tf
   geometry_msgs::TransformStamped odom_trans;
   odom_trans.header.stamp = currentTime;
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = "base_link";

   odom_trans.transform.translation.x = x;
   odom_trans.transform.translation.y = y;
   odom_trans.transform.translation.z = 0.0;
   odom_trans.transform.rotation = odom_quat;

   //send the transform
   odom_broadcaster.sendTransform(odom_trans);

   //next, we'll publish the odometry message over ROS
   nav_msgs::Odometry odom;
   odom.header.stamp = currentTime;
   odom.header.frame_id = "odom";

   //set the position
   odom.pose.pose.position.x = x;
   odom.pose.pose.position.y = y;
   odom.pose.pose.position.z = 0.0;
   odom.pose.pose.orientation = odom_quat;

   //set the velocity
   odom.child_frame_id = "base_link";
   odom.twist.twist.linear.x = vx;
   odom.twist.twist.linear.y = vy;
   odom.twist.twist.angular.z = vth;

   //publish the message
   odom_pub.publish(odom);
}

void calculOdometry(const robair_demo::encoderData::ConstPtr& msg)
{
   ROS_INFO("I heard: [%d, %d]", msg->wheelLeft,  msg->wheelRight);
   currentTime = ros::Time::now();
   tickG = msg->wheelLeft;
   tickD = msg->wheelRight;
   int32_t dtickG = tickG - tickGPrec;//vg
   int32_t dtickD = tickD - tickDPrec;//vd
   double dThetaTicks = -(dtickG - dtickD)/2; 
   double dDeltaTicks = (dtickG + dtickD)/2;//v
   ROS_INFO("dDelta = %lf, dTheta = %lf",  dDeltaTicks,  dThetaTicks);
   double vth = (dThetaTicks*TICK_TO_RAD);// en Rad

   vx = (double)dDeltaTicks*TICKS_TO_M*cos(th);
   vy = (double)dDeltaTicks*TICKS_TO_M*sin(th);

   x +=vx;
   y +=vy;
   th += vth;

   ROS_INFO("x= %lf, y = %lf, th = %lf",x,y,th); 

   doPublish = true;
 
   tickDPrec = tickD;
   tickGPrec = tickG;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "odometry");

   ros::NodeHandle n;

   ros::Publisher odom_pub;
   tf::TransformBroadcaster odom_broadcaster;
   // a publisher of /odom
   odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
   // subscribing to /encoder
   ros::Subscriber sub = n.subscribe("encoder", 1000, calculOdometry);

   ros::Rate loop_rate(50);

   ROS_INFO("TICK_TO_RAD : %lf", TICK_TO_RAD);
   ROS_INFO("TICKS_TO_M : %lf", TICKS_TO_M);

   while (ros::ok())
   {
      if (doPublish)
      {
         doPublish = false;
         publish(odom_pub, odom_broadcaster);
      }
      ros::spinOnce();
      loop_rate.sleep();
   }

   //ros::spin();
   
   return 0;
}
