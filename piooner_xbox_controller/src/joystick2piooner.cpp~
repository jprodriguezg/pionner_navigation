#include <iostream>
#include <string>
#include <math.h> 
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

# define PI 3.14159265358979323846


double linear_vel, linear_scale, angular_vel, angular_scale;
int enable_motors, disable_motors;
geometry_msgs::Twist velocityMsg;

std::vector<double> last_vel (2,0);


void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){

	// Update velocity joystick commands

	velocityMsg.linear.x = linear_scale*msg->axes[1];
	// Linear velocity limits 
	velocityMsg.linear.x=std::min(0.8,velocityMsg.linear.x);
	velocityMsg.linear.x=std::max(-0.8,velocityMsg.linear.x);
	
	velocityMsg.angular.z = angular_scale*msg->axes[3];
	// Angular velocity limits 
	velocityMsg.angular.z=std::min(0.8,velocityMsg.angular.z);
	velocityMsg.angular.z=std::max(-0.8,velocityMsg.angular.z);

	// Update enable/disable commands
	enable_motors = msg->buttons[3];
	disable_motors = msg->buttons[1];

	//std::cout << "Joy outpus " << msg->axes[1] << "  " << msg->axes[3] << std::endl;

  return;
}

double robotJoy_stop(double current_vel){

	if (current_vel<= 0.05 && current_vel >= -0.05)
		current_vel = 0;

	return current_vel;
}

int main(int argc, char** argv){

ros::init(argc, argv, "joystick2piooner_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

linear_scale = 1;
angular_scale = 1;
last_vel[0] = velocityMsg.linear.x; 
last_vel[1] = velocityMsg.angular.z;

std_srvs::Empty motors_srv;
ros::Subscriber joy_sub_=nh_.subscribe("joystick_status", 10, joyCallback);
ros::Publisher vel_piooner_pub_=nh_.advertise<geometry_msgs::Twist>("piooner_velocity_topic", 1);
ros::ServiceClient piooner_enable_motors_ =  nh_.serviceClient<std_srvs::Empty>("enable_motors_service");
ros::ServiceClient piooner_disable_motors_ =  nh_.serviceClient<std_srvs::Empty>("disable_motors_service");

	while (ros::ok()){

	if (last_vel[0] != velocityMsg.linear.x || last_vel[1] != velocityMsg.angular.z ){

		velocityMsg.linear.x = robotJoy_stop(velocityMsg.linear.x);
		velocityMsg.angular.z = robotJoy_stop(velocityMsg.angular.z);

		// Publish the new velocities of the Robot
		vel_piooner_pub_.publish(velocityMsg);

		// Save the last velocities 
		last_vel[0] = velocityMsg.linear.x; 
		last_vel[1] = velocityMsg.angular.z;
		}

	if (enable_motors == 1)
		piooner_enable_motors_.call(motors_srv);

	if (disable_motors == 1)
		piooner_disable_motors_.call(motors_srv);

	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
	}

return 0;
}
