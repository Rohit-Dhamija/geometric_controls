# include "ros/ros.h"
# include "prius_msgs/Control.h"
# include "nav_msgs/Odometry.h"
# include "nav_msgs/Path.h"
# include <math.h>
# include <cmath>

# include <iostream>           

using namespace std;
using namespace nav_msgs;

float lcar = 3, Vpid = 3;            // Vpid is Target speed
float kefa = 0.05;                   // stanley gain parameter 
float kp = 0.6, ki = 0.6, kd = 1;    // PID gains 


typedef struct
{
 float X;
 float Y;
}point;

typedef struct
{
 float Qx;
 float Qy;
 float Qz;
 float Qw;
}quarter;


point path[1500];
quarter pathq[1500]; 


float x,y,xf,yf,Vx,Vy;                      // (xf,yf) are coordinates of the centre of the front axle  whereas (Vx,Vy) is the current Velocity of the car
float qx,qy,qz,qw,yaw_car,qX,qY,qZ,qW;      // quaterniions of car location and closest path point
float efa;                                  // efa is distance from closest path point

float sum_e,e_prev,throttle = 0,steer = 0;

int min_index = 0;

int find_pt()                               // finding closest point
{ 
 int i;
 efa = sqrt(pow( (path[0].X-xf),2) + pow((path[0].Y-yf),2) );
   
 for (i=min_index+1; i < 1159; i++)
     {
      float dist = sqrt(pow( (path[i].X-xf),2) + pow((path[i].Y-yf),2) );
     
      if (dist < efa)         
          efa = dist;
          min_index = i;
     }
     
 
 qX = pathq[min_index].Qx;
 qY = pathq[min_index].Qy;
 qZ = pathq[min_index].Qz;
 qW = pathq[min_index].Qw;
 
 return min_index;            
}





float find_delta()                              // finding steer angle (delta)
{ 
 float delta = 0;
  
 
 float yaw_point = atan2(2*(qX*qY+qW*qZ),1-2*(qY*qY+qZ*qZ));     // finding yaw from quaternions 
 
 float cross = Vx*(path[min_index].Y-y) - Vy*(path[min_index].X-x);
 


float V = sqrt(Vx*Vx + Vy*Vy);                                                          
if (cross > 0)
{
delta = (yaw_point-yaw_car) + atan (kefa*efa/(V+1e-9));
}
else
{efa = -efa;
delta = (yaw_point-yaw_car) + atan (kefa*efa/(V+1e-9));

} 
 return delta;
}




float pid ()     // Velocity controlled using PID controller
{ 
 float V = sqrt(Vx*Vx + Vy*Vy);
 cout << "Velocity = " << V << endl; 
 float e = Vpid - V;

 sum_e = sum_e + e;

 
 float THROTTLE = kp*e + ki*sum_e + kd*(e - e_prev);

 e_prev = e;

 if (THROTTLE > 1){THROTTLE = 1;}     
 if (THROTTLE < 0){ THROTTLE = 0;}      
 throttle = THROTTLE;
}
  
 



void head_func()
{ 
                                                         
 int index = find_pt();


 float delta = find_delta();
 
 pid();
 
 
 steer = delta *180*7/22;

 if (steer > 40){ steer = 40;}                          // 40 the maximum steering angle for Toyota Prius
 if (steer < -40){ steer = -40;}     

 steer = steer/40;

} 




void odomfunc(const OdometryConstPtr& odom)
{ 
  x = odom->pose.pose.position.x;
  y = odom->pose.pose.position.y;
  Vx= odom->twist.twist.linear.x;
  Vy= odom->twist.twist.linear.y; 

  qx= odom->pose.pose.orientation.x; 
  qy= odom->pose.pose.orientation.y;
  qz= odom->pose.pose.orientation.z;
  qw= odom->pose.pose.orientation.w;

 
  yaw_car = atan2(2*(qx*qy+qw*qz),1-2*(qy*qy+qz*qz));   //Finding yaw from quaternions 


  
  xf = x + lcar*cos(yaw_car);
  yf = y + lcar*sin(yaw_car);
} 




void pathfunc(const PathConstPtr& Path)
{
 
 int i;

 for (i=1159; i > -1 ; i--)
     {
      path[i].X = Path->poses[i].pose.position.x;
      path[i].Y = Path->poses[i].pose.position.y;

      pathq[i].Qx = Path->poses[i].pose.orientation.x;
      pathq[i].Qy = Path->poses[i].pose.orientation.y;
      pathq[i].Qz = Path->poses[i].pose.orientation.z;
      pathq[i].Qw = Path->poses[i].pose.orientation.w;
     }
 
 head_func();

} 




int main (int argc, char **argv)
{
 ros::init(argc,argv,"SL");
 
 ros::NodeHandle n;

 ros::Subscriber sub1 = n.subscribe("/base_pose_ground_truth", 1000, odomfunc);

 ros::Subscriber sub2 = n.subscribe("/astroid_path", 1000, pathfunc);

 ros::Publisher instance = n.advertise<prius_msgs::Control>("/prius",1000);
 prius_msgs::Control msg;

 ros::Rate loop_rate(10);

 while (ros::ok())
{
 msg.throttle = throttle;
 msg.brake = 0.0; 
 msg.steer = steer;
 msg.shift_gears =2;        //forward gear

 instance.publish(msg);
 ros::spinOnce();
 loop_rate.sleep();
}

 return 0;
}
