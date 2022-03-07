# include "ros/ros.h"
# include "prius_msgs/Control.h"
# include "nav_msgs/Odometry.h"
# include "nav_msgs/Path.h"
# include <math.h>
# include <cmath>
# include <cstdlib>
# include <iostream>           

using namespace std;           
using namespace nav_msgs;


float  lcar = 3, Vpid = 5;                   // Vpid is Target speed 
float kp = 0.8, ki =0.00001, kd =1;          // PID gains   
double kld = 1.25;                           //lookout distance                                           


typedef struct
{
 float X;
 float Y;
}point;


point path[1500];                                                                             

float x,y,Vx,Vy,X,Y;                      // (x,y) and (Vx,Vy) will have the odometry and current Velocity of the car respectively whereas (X,Y) will have location of the selected path point
float qx,qy,qz,qw;                        // quaternions of the car orientation

float sum_e,e_prev,throttle=0.0,steer=0.0;                         



int find_pt()                             // finding point on the path at a distance of kld
{
 float dist;
 int i = 1199;
 while (i>-1)
     { 
      dist = sqrt(pow( (path[i].X-x),2) - pow((path[i].Y-y),2) );
      float ld = kld*sqrt(Vx*Vx + Vy*Vy);
      if (abs(dist-ld) < 0.5 )
         {
          break;
         }
      else 
         {
          i--;
         }
     }
return i;
}


float find_dist (int i)
{
 
 float yaw = atan2(2*(qx*qy+qw*qz),1-2*(qy*qy+qz*qz));  //Finding yaw from quaternions 

 X = path[i].X;
 Y = path[i].Y;

 float ed = (tan(yaw)*X - Y + y - x*tan(yaw)) / (sqrt(tan(yaw)*tan(yaw) + 1));   // dist. of a point from a line

 return abs(ed);
}


float pid ()       // Velocity controlled using PID controller
{ 
 float V = sqrt(Vx*Vx + Vy*Vy);

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
 
 float ed = find_dist(index);
 
 float cross = Vx*(Y-y) - Vy*(X-x);
 if (cross < 0)
   { ed = ed*(-1); }

 if (sqrt(Vx*Vx + Vy*Vy) != 0 )                                
  {   
  float ld = kld*sqrt(Vx*Vx + Vy*Vy);
  float delta = atan (2*lcar*ed/(ld*ld));
  steer = delta *180*7/22;
  } 

 if (steer > 40){ steer = 40;}          // 40 the maximum steering angle for Toyota Prius                                                       
 if (steer < -40){ steer = -40;}      

 steer = steer/40;                                       

 pid();  
 cout << "VELOCITY (in m/s) = " << setprecision(3)  << sqrt(Vx*Vx + Vy*Vy) << endl;
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
} 

void pathfunc(const PathConstPtr& Path)
{
 int i;

 for (i=0;  i < 1199 ; i++)
     {
      path[i].X = Path->poses[i].pose.position.x;
      path[i].Y = Path->poses[i].pose.position.y;
     }
 
 head_func();

} 

int main (int argc, char **argv)
{
 ros::init(argc,argv,"PP");
 
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
 msg.shift_gears =2;  //forward gear                                                                  
 instance.publish(msg);

 ros::spinOnce();
 loop_rate.sleep();
 
 }

 

 return 0;
}
