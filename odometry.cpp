#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <iostream>



class Odometry {
public:
// Construst a new odometry object and hook up this ROS node
// to the simulated robot's velocity control and laser topics
Odometry(ros::NodeHandle& nh) :
fsm(FSM_MOVE_FORWARD),
rotateStartTime(ros::Time::now()),
rotateDuration(0.f) {
// Initialize random time generator
srand(time(NULL));

// Advertise a new publisher for the turtlebot robot's velocity command topic
// (the second argument indicates that if multiple command messages are in
//  the queue to be sent, only the last command will be sent)
commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);

// Subscribe to the turtlebot robot's laser scan topic and tell ROS to call
// this >commandCallback() whenever a new message is published on that topic
laserSub = nh.subscribe("scan", 1, &Odometry::commandCallback, this);
};
// Send a velocity command 
void move(double linearVelMPS, double angularVelRadPS) {
geometry_msgs::Twist msg; // The default constructor will set all commands to 0
msg.linear.x = linearVelMPS;
msg.angular.z = angularVelRadPS;
commandPub.publish(msg);
};


// Process the incoming laser scan message
void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
float closestRange = msg->ranges[minIndex];
for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
if (msg->ranges[currIndex] <closestRange) {closestRange = msg->ranges[currIndex];
}
}

    std::cout << "\nWhat would you like me to do \n";
    std::cout << "a)Square\nb)Line\nc)Rotate \n";
    std::cin >> patternChoice;
    
    
    
    if (patternChoice == 'a'){
        std::cout <<"Square";
        
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <  tend;){
            move(FORWARD_SPEED_MPS, 0);
            t1 = ros::Time::now();
        }
        
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(0, ROTATE_SPEED_RADPS);
            t1 = ros::Time::now();
        }
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(FORWARD_SPEED_MPS, 0);
            t1 = ros::Time::now();
        }
        
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(0, ROTATE_SPEED_RADPS);
            t1 = ros::Time::now();
        }
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(FORWARD_SPEED_MPS, 0);
            t1 = ros::Time::now();
        }
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(0, ROTATE_SPEED_RADPS);
            t1 = ros::Time::now();
        }
        
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(FORWARD_SPEED_MPS, 0);
            t1 = ros::Time::now();
        }
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(0, ROTATE_SPEED_RADPS);
            t1 = ros::Time::now();
        }
        
        
    }	
      else if (patternChoice == 'b'){
        std::cout <<"Line";
        
        tend = ros::Time::now() + ros::Duration(4);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(FORWARD_SPEED_MPS, 0);
            t1 = ros::Time::now();
        }
    
        
    
    }	
      else if (patternChoice == 'c'){
        std::cout <<"Rotate";
        tend = ros::Time::now() + ros::Duration(8/6);
        for (ros::Time t1 = ros::Time::now(); t1 <tend;){
            move(0, ROTATE_SPEED_RADPS);
            t1 = ros::Time::now();
        }
    }
    else{
        std::cout <<"Not a valid choice please select again";
    }



};



// Main FSM loop for ensuring that ROS messages are
// processed in a timely manner, and also for sending
// velocity controls to the simulated robot based on the FSM state
void spin() {
ros::Rate rate(30); // Specify the FSM loop rate in Hz
while(ros::ok()) { // Keep spinning loop until user presses Ctrl+C
   
    
    
  

ros::spinOnce(); 
// Need to call this function often to allow ROS to process incoming messages
rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
}
};


enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
// Tunable parameters
const static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
const static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
const static float PROXIMITY_RANGE_M = 0.75; // Should be smaller than sensor_msgs::LaserScan::range_max
const static double FORWARD_SPEED_MPS = 0.25;
const static double ROTATE_SPEED_RADPS = M_PI/8;
char patternChoice;



protected:
ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
enum FSM fsm; // Finite state machine for the random walk algorithm
ros::Time rotateStartTime; // Start time of the rotation
ros::Duration rotateDuration; // Duration of the rotation
ros::Time tend; // used to control how long the loop runs

};


int main(int argc, char**argv) {
ros::init(argc, argv, "odometry"); // Initiate new ROS node named "random_walk"
ros::NodeHandle n;
Odometry walker(n); // Create new random walk object
walker.spin(); // Execute FSM loop
return 0;
}
