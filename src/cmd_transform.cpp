#include "ros/ros.h"
#include "robair_demo/Command.h"
#include <geometry_msgs/Twist>
#include <math.h> 
#include <stdint.h>

#define TICKS_PAR_TOUR 980//588
#define DIAMETRE_ROUE  0.125 //en m
#define ENTRE_ROUES 0.325 //en m
#define TICK_TO_RAD ((DIAMETRE_ROUE*2*M_PI)/(TICKS_PAR_TOUR*ENTRE_ROUES))
#define TICKS_TO_M ((M_PI*DIAMETRE_ROUE)/TICKS_PAR_TOUR)

#define COEF_TURN 28   //==28 (unité moteur)/1(rad/s)
#define COEF_SPEED 160 // ==48(unités moteur)/0.3(m/s)

//int32_t tickG = 0;
//int32_t tickD = 0;
//int32_t tickGPrec = 0;
//int32_t tickDPrec = 0;

//double x=0,y=0,th=0;
//double vx=0, vy=0, vth=0;

uint8_t speed1 = 128;
uint8_t turn = 0;

double posX=0,posY=0;
double theta = 0;

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
   odom_pub.publish(cmd);
}

void getPose(const geometry_msgs::PoseWithCovarianceStamped:ConstPtr& msg)
{
   //récupère la position du robot posX posY theta
   posX=(double)(msg->pose.pose.position.x);
   posY=(double)(msg->pose.pose.position.y);
   tf::Pose pose;
   tf::poseMsgToTF(msg->pose.pose , pose);
   theta=tf::getYaw(pose.getRotation());

}


void calculCommand(const geometry_msgs::Twist::ConstPtr& msg)
{
   //transforme la commande de AMCL en speed/turn pour le moteur
   ROS_INFO("I heard: [%lf, %lf, %lf]", msg->linear.x, msg->angular.z);

   double vx = msg->linear.x;
   //double vy = msg->linear.y;
   double vt = msg->angular.z;  //max 1 rad/sec

   speed1 = 128 + COEF_SPEED * vx ;
   turn = COEF_TURN * vt + 128;


   ROS_INFO("speed1 = %d, turn = %d", speed1, turn);

   doPublish = true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "cmd_transform");

   ros::NodeHandle n;

   ros::Publisher cmd_pub;
   // a publisher of /cmd
   cmd_pub = n.advertise<robair_demo::Command>("cmd", 50);
   // subscribing to /encoder
   ros::Subscriber sub = n.subscribe("cmd_vel", 1000, calculCommand);
   ros::Subscriber sub2 = n.subscribe("amcl_pose", 1000, getPose);

   ros::Rate loop_rate(100);

   ROS_INFO("TICK_TO_RAD : %lf", TICK_TO_RAD);
   ROS_INFO("TICKS_TO_M : %lf", TICKS_TO_M);

   while (ros::ok())
   {
      if (doPublish)
      {
         doPublish = false;
         publish(cmd_pub);
      }
      ros::spinOnce();
      loop_rate.sleep();
   }

   //ros::spin();
   
   return 0;
}
