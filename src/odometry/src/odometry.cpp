#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib> // Needed for rand()
#include <iostream>
#include <termios.h>//needed for getch function



class Odometry {
public:
// Construst a new odometry object and hook up this ROS node
// to the robot's velocity control and laser topics
Odometry(ros::NodeHandle& nh) {

// Advertise a new publisher for the turtlebot robot's velocity command topic
// (the second argument indicates that if multiple command messages are in
//  the queue to be sent, only the last command will be sent)
commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
}

// Send a velocity command 
void move(double linearVelMPS, double angularVelRadPS) {
geometry_msgs::Twist msg; // The default constructor will set all commands to 0
msg.linear.x = linearVelMPS;
msg.angular.z = angularVelRadPS;
commandPub.publish(msg);
};


/*The following function was taken from 
 *https://github.com/sdipendra/ros-projects/blob/master/src/keyboard_non_blocking_input/src/keyboard_non_blocking_input_node.cpp*/
char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0)
		ROS_INFO("no_key_pressed");
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}


void spin() {
ros::Rate rate(1); // Specify the FSM loop rate in Hz
while(ros::ok()) { // Keep spinning loop until user presses Ctrl+C
   std::cout << "\nWhat would you like me to do \n";
    std::cout << "a)Square\nb)Line\nc)Rotate \n";
    patternChoice = getch();
    
    
    
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
    
    
  

ros::spinOnce(); 
// Need to call this function often to allow ROS to process incoming messages
rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
}
};




// Tunable parameters
const static double FORWARD_SPEED_MPS = 0.25;
const static double ROTATE_SPEED_RADPS = M_PI/8;
char patternChoice;



protected:
ros::Publisher commandPub; // Publisher to the robot's velocity command topic
ros::Time tend; // used to control how long the loop runs


};


int main(int argc, char**argv) {
ros::init(argc, argv, "odometry"); // Initiate new ROS node named "odometry"
ros::NodeHandle n;
Odometry walker(n); // Create new odometry object
walker.spin(); // Execute FSM loop
return 0;
}
