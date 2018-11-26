#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib> // Needed for rand()
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>

geometry_msgs::Pose2D current_pose;
ros::Publisher poseUpdate;

class BasicMotion {
public:
// Construst a new odometry object and hook up this ROS node
// to the robot's velocity control and laser topics
BasicMotion(ros::NodeHandle& nh) {

// Advertise a new publisher for the turtlebot robot's velocity command topic
// (the second argument indicates that if multiple command messages are in
//  the queue to be sent, only the last command will be sent)
commandPub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

poseSub = nh.subscribe("odom", 1, \
      &BasicMotion::poseCallback, this);

ekfposeSub = nh.subscribe("robot_pose_ekf/odom",1, \
        &BasicMotion::ekfPoseCallback, this);

poseUpdate = nh.advertise<geometry_msgs::Pose2D>("turtlebot_pose2d", 1);
}

// Send a velocity command 
void move(double linearVelMPS, double angularVelRadPS) {
geometry_msgs::Twist msg; // The default constructor will set all commands to 0
msg.linear.x = linearVelMPS;
msg.angular.z = angularVelRadPS;
ROS_INFO_STREAM(msg);
commandPub.publish(msg);
};

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    current_pose.x = -msg->pose.pose.position.y;
    current_pose.y = msg->pose.pose.position.x;
    current_pose.theta = tf::getYaw(msg->pose.pose.orientation);
    poseUpdate.publish(current_pose);
  };

  void ekfPoseCallback (const nav_msgs::Odometry::ConstPtr& msg){
  
  }
  
  
  
  void transform (double distance){
      tnew = ros::Time::now() + ros::Duration((1/FORWARD_SPEED_MPS) * distance);
      for (ros::Time t1 = ros::Time::now(); t1 <  tnew;){
            move(FORWARD_SPEED_MPS, 0);
            t1 = ros::Time::now();
            
      }
  }
  
  void rotateRel(double angle){
      tnew = ros::Time::now() + ros::Duration((1/ROTATE_SPEED_RADPS) * angle);
      for (ros::Time t1 = ros::Time::now(); t1 <  tnew;){
            move(0, ROTATE_SPEED_RADPS);
            t1 = ros::Time::now();
            
      }
  }
  
   void rotateAbs(double angle, double angleStart){
       std::cout << "Moving from " << angleStart << " to " << angle;
       double deltaAngle = angle - angleStart;
       std::cout << "\n" << deltaAngle;
       if (deltaAngle < 0){
           deltaAngle = deltaAngle + 2 * M_PI;
       }
      tnew = ros::Time::now() + ros::Duration((1/ROTATE_SPEED_RADPS) * (deltaAngle));
      for (ros::Time t1 = ros::Time::now(); t1 <  tnew;){
            move(0, ROTATE_SPEED_RADPS);
            t1 = ros::Time::now();
            
      }
  }

void spin() {
ros::Rate rate(10); // Specify the FSM loop rate in Hz
while(ros::ok()) { // Keep spinning loop until user presses Ctrl+C
    ros::spinOnce(); 
    
    
    
    std::cout << "\nWhat would you like me to do \n";
    std::cout << "a)Transform\nb)Rotate Relative\nc)Rotate Absolute\n";
    std::cin >> userChoice;

    if (userChoice == "a"){
        std::cout << "\n\n\nEnter a distance (in meters)\n";
        std::cin >> d;     
        if (d > 0){
            //TODO figure out what is wrong with this
            ROS_INFO_STREAM("(" << current_pose.x << ", " << current_pose.y << ") and heading " << current_pose.theta);
            double startingX = current_pose.x;
            double startingTheta = current_pose.theta;
            while (ros::ok() && current_pose.x < startingX + cos(startingTheta)){
                ROS_INFO_STREAM("(" << current_pose.x << ", " << current_pose.y << ")");
                move(FORWARD_SPEED_MPS, 0);
                ros::spinOnce();
                rate.sleep();
            }
        }
        else{
            std::cout << "Enter a positive distance\n";
        }
        
    }
    else if (userChoice == "b"){
        
        //TODO Fix these cases for angles over 90deg
        
        
        std::cout << "\n\n\nEnter an angle (in degrees)\n";
        std::cin >> angle;
        double startingTheta = current_pose.theta;
        if (angle < 90){
            angle = angle * M_PI/180;
            while (ros::ok() && current_pose.theta < startingTheta + angle){
                ROS_INFO_STREAM(current_pose.theta);
                move(0, ROTATE_SPEED_RADPS);
                ros::spinOnce();
                rate.sleep();
            }}
            else if (angle < 180){
                angle = angle * M_PI/180;
                double newAngle = angle - 90;
                while (ros::ok() && current_pose.theta < startingTheta + 90){
                    ROS_INFO_STREAM(current_pose.theta);
                    move(0, ROTATE_SPEED_RADPS);
                    ros::spinOnce();
                    rate.sleep();
                }
                startingTheta = current_pose.theta;
                while (ros::ok() && current_pose.theta < startingTheta + newAngle){
                    ROS_INFO_STREAM(current_pose.theta);
                    move(0, ROTATE_SPEED_RADPS);
                    ros::spinOnce();
                    rate.sleep();
                }                
        }
    }
    
    else if (userChoice == "c"){
        std::cout << "\n\n\nEnter an angle (in degrees)\n";
        std::cin >> angle;
        angle = angle * M_PI/180;
        
        if (angle <= M_PI && angle >= 0){
            angle = angle;
        }
        else if (angle <= 2 * M_PI && angle >= 0){
            angle = angle - 2 * M_PI;
        }
        else{
            std::cout << "\n********Not a valid angle please try again********\n";
            angle = current_pose.theta;
        }
        while (ros::ok() && (current_pose.theta > angle + 0.5 || current_pose.theta < angle - 0.5)){
                ROS_INFO_STREAM(current_pose.theta);
                move(0, ROTATE_SPEED_RADPS);
                ros::spinOnce();
                rate.sleep();
            }
        
    }
  
    
  


// Need to call this function often to allow ROS to process incoming messages
rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
}
};




// Tunable parameters
const static double FORWARD_SPEED_MPS = 0.25;
const static double ROTATE_SPEED_RADPS = M_PI/16;




protected:
ros::Publisher commandPub; // Publisher to the robot's velocity command topic
ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic
ros::Publisher poseUpdate; //publish pose as running
ros::Time tend; // used to control how long the loop runs
double x; 
double y;
double heading; // in radians,
std::string userChoice;
double d;
double angle;
ros::Time tnew;
ros::Subscriber ekfposeSub;
};


int main(int argc, char**argv) {
ros::init(argc, argv, "basicmotion"); // Initiate new ROS node named "basicmotion"

ros::NodeHandle n;

BasicMotion walker(n); // Create new object
walker.spin(); // Execute FSM loop
return 0;
}
