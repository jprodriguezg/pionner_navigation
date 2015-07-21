#include <iostream>
#include <string>
#include <vector>
#include <math.h> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <drone_control_msgs/send_control_data.h>

# define PI           3.14159265358979323846
std::vector<double> Drone_info(4,0);

// Define callbacks
void hasReceivedModelState(const geometry_msgs::PoseStamped::ConstPtr& msg){

	// Obtaining drone info 
  	Drone_info[0] = msg->pose.position.x; 
	Drone_info[1] = msg->pose.position.y;
	Drone_info[2] = msg->pose.position.z;

  return;
}

void new_quadrant(double quadrant[4][2], std::vector<double> central_point, double range){

	quadrant[0][0] = central_point[0]-range/2;
	quadrant[0][1] = central_point[1]+range/2;
	quadrant[1][0] = central_point[0]+range/2;
	quadrant[1][1] = central_point[1]+range/2;
	quadrant[2][0] = central_point[0]+range/2;
	quadrant[2][1] = central_point[1]-range/2;
	quadrant[3][0] = central_point[0]-range/2;
	quadrant[3][1] = central_point[1]-range/2;

}

int nearest_vertex(double quadrant[4][2]){
	int index = 0;
	double new_rho, rho = 1000;
	for (int i=0; i<4; i++){
		new_rho = sqrt(pow(quadrant[i][0]-Drone_info[0],2)+pow(quadrant[i][1]-Drone_info[1],2));
		if (new_rho < rho){
			index = i;
		} 		
	} 
	
	return index;
}

	
int main(int argc, char** argv){
    
ros::init(argc, argv, "set_waypoints_node");
ros::NodeHandle nh_;
ros::NodeHandle nhp_("~");
ros::Rate rate(20.0);

ros::Subscriber optitrack_sub_=nh_.subscribe("drone_info_topic", 10, hasReceivedModelState);
ros::Publisher waypoint_info_pub_=nh_.advertise<drone_control_msgs::send_control_data>("waypoint_topic", 1);


double rho, base_time = 0.0, delta_time = 0.0, range = 2.0;
std::vector<double> central_point (2,0), final_pose (2,0), emergency_pose(2,0);
drone_control_msgs::send_control_data waypoint_publish;

bool flag = true, emergency = false;
int index = 0;
double quadrant[4][2];
// Original definition of quadrant
/* double quadrant[4][2] = {{central_point[0]-range/2, central_point[1]+range/2},
				{central_point[0]+range/2, central_point[1]+range/2},
				{central_point[0]+range/2, central_point[1]-range/2},
				{central_point[0]-range/2, central_point[1]-range/2}};*/

	while (ros::ok()){

		nhp_.getParam("emergency_position",emergency_pose);
		nhp_.getParam("emergency",emergency);

		if(emergency == true){
			waypoint_publish.position.x = emergency_pose[0];
			waypoint_publish.position.y = emergency_pose[1];
			flag = true;
			nhp_.setParam("delta_time", 0.0);
		}
		
		else{
			

			if (flag == true){
				nhp_.getParam("range",range);
				nhp_.getParam("central_point",central_point);
				nhp_.getParam("final_position",final_pose);
				nhp_.getParam("delta_time",delta_time);

				if (delta_time != 0.0){

					new_quadrant(quadrant,central_point,range);
					index = nearest_vertex(quadrant);    
					flag = false;
					base_time = ros::Time::now().toSec();
				}
			}

			else{

				if (base_time+delta_time > ros::Time::now().toSec()){
					rho = sqrt(pow(quadrant[index][0]-Drone_info[0],2)+pow(quadrant[index][1]-Drone_info[1],2));
					if (rho < 0.2){
						if (index<3){
							index++;
						}
						else{
							index = 0;
						}
					}
					waypoint_publish.position.x = quadrant[index][0];
					waypoint_publish.position.y = quadrant[index][1];
				}
				else{
					waypoint_publish.position.x = final_pose[0];
					waypoint_publish.position.y = final_pose[1];
					flag = true;
					nhp_.setParam("delta_time", 0.0);
					base_time = 0.0;
				}
			}
		}
	
		waypoint_publish.position.z = 1.0;
		waypoint_info_pub_.publish(waypoint_publish);

		/*
		std::cout <<delta_time<<std::endl;
		std::cout <<rho<<std::endl;
		std::cout <<ros::Time::now()<<std::endl;*/
		

		
	
	   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
	    	rate.sleep();
	}

 return 0;
}
