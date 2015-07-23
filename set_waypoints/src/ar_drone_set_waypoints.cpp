#include <iostream>
#include <string>
#include <vector>
#include <math.h> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <drone_control_msgs/send_control_data.h>
#include <robot_cooperation_project_msgs/waypoints_info.h>
#include <robot_cooperation_project_msgs/sector_info.h>

# define PI           3.14159265358979323846
std::vector<double> Drone_info(4,0);
robot_cooperation_project_msgs::waypoints_info waypoints_info;

// Define callbacks
void hasReceivedModelState(const geometry_msgs::PoseStamped::ConstPtr& msg){

	// Obtaining drone info 
  	Drone_info[0] = msg->pose.position.x; 
	Drone_info[1] = msg->pose.position.y;
	Drone_info[2] = msg->pose.position.z;
	waypoints_info.position.x = Drone_info[0];
	waypoints_info.position.y = Drone_info[1];
	waypoints_info.position.z = Drone_info[2];

  return;
}

void fill_waypoints_info(bool emergency, bool stop_mission, std::vector<double> final_pose, std::vector<double> emergency_pose, double rho, double minimum_radius, double flight_time, drone_control_msgs::send_control_data waypoint_publish){
	
	waypoints_info.emergency_status = emergency;
	waypoints_info.stop_mission_status = stop_mission;
	waypoints_info.target_point.x = waypoint_publish.position.x;
	waypoints_info.target_point.y = waypoint_publish.position.y;
	waypoints_info.target_point.z = 1.0;
	waypoints_info.final_position.x = final_pose[0];
	waypoints_info.final_position.y = final_pose[1];
	waypoints_info.final_position.z = 1.0;
	waypoints_info.emergency_position.x = emergency_pose[0];
	waypoints_info.emergency_position.y = emergency_pose[1];
	waypoints_info.emergency_position.z = 1.0;
	waypoints_info.flight_time = flight_time;
	waypoints_info.target_distance = rho;
	waypoints_info.minimum_radius = minimum_radius;

}

void new_quadrant(double quadrant[4][2], std::vector<double> central_point, double range){

	quadrant[0][0] = central_point[0]-range/2;
	waypoints_info.sector.vertex_1[0] = quadrant[0][0];
	quadrant[0][1] = central_point[1]+range/2;
	waypoints_info.sector.vertex_1[1] = quadrant[0][1];
	quadrant[1][0] = central_point[0]+range/2;
	waypoints_info.sector.vertex_2[0] = quadrant[1][0];
	quadrant[1][1] = central_point[1]+range/2;
	waypoints_info.sector.vertex_2[1] = quadrant[1][1];
	quadrant[2][0] = central_point[0]+range/2;
	waypoints_info.sector.vertex_3[0] = quadrant[2][0];
	quadrant[2][1] = central_point[1]-range/2;
	waypoints_info.sector.vertex_3[1] = quadrant[2][1];
	quadrant[3][0] = central_point[0]-range/2;
	waypoints_info.sector.vertex_4[0] = quadrant[3][0];
	quadrant[3][1] = central_point[1]-range/2;
	waypoints_info.sector.vertex_4[1] = quadrant[3][1];

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
ros::Publisher waypoints_pub_=nh_.advertise<drone_control_msgs::send_control_data>("waypoint_topic", 1);
ros::Publisher waypoint_info_pub_=nh_.advertise<robot_cooperation_project_msgs::waypoints_info>("waypoint_info_topic", 1);


double rho, minimum_radius = 0.2, base_time = 0.0, flight_time = 0.0, range = 2.0;
std::vector<double> central_point (2,0), final_pose (2,0), emergency_pose(2,0);
drone_control_msgs::send_control_data waypoint_publish;
waypoint_publish.position.z = 1.0;
bool flag_parameters = true, flag_stop = true, emergency = false, stop_mission = false;
int index = -1, ant_index = -1;


// Pre definition of quadrant matrix
nhp_.getParam("range",range);
nhp_.getParam("central_point",central_point);
double quadrant[4][2] = {{central_point[0]-range/2, central_point[1]+range/2},
				{central_point[0]+range/2, central_point[1]+range/2},
				{central_point[0]+range/2, central_point[1]-range/2},
				{central_point[0]-range/2, central_point[1]-range/2}};

// Pre fill of the sector structure (waypoints_info)
waypoints_info.sector.central_point.resize(2);
waypoints_info.sector.vertex_1.resize(2);
waypoints_info.sector.vertex_2.resize(2);
waypoints_info.sector.vertex_3.resize(2);
waypoints_info.sector.vertex_4.resize(2);
waypoints_info.sector.edge_size = range;
waypoints_info.sector.central_point[0] = central_point[0];
waypoints_info.sector.central_point[1] = central_point[1];

	while (ros::ok()){

		nhp_.getParam("emergency_position",emergency_pose);
		nhp_.getParam("emergency",emergency);
		nhp_.getParam("stop_mission",stop_mission);
		nhp_.getParam("minimum_radius",minimum_radius);

		if(emergency == true || stop_mission == true){
			if(emergency == true){
				waypoint_publish.position.x = emergency_pose[0];
				waypoint_publish.position.y = emergency_pose[1];
			}
			else{
				waypoint_publish.position.x = Drone_info[0];
				waypoint_publish.position.y = Drone_info[1];
			}

			if (flag_stop == true){ // To call the client only once
				waypoints_pub_.publish(waypoint_publish);
				nhp_.setParam("flight_time", 0.0);
				flight_time = 0.0;
				base_time = 0.0;
				waypoints_info.mission_time = base_time;
				flag_parameters = true;
				flag_stop == false;

				}	
		}
		
		else{	
			flag_stop = true;
			if (flag_parameters == true){
				nhp_.getParam("range",range);
				nhp_.getParam("central_point",central_point);
				nhp_.getParam("final_position",final_pose);
				nhp_.getParam("flight_time",flight_time);

				if (flight_time != 0.0){

					new_quadrant(quadrant,central_point,range);
					index = nearest_vertex(quadrant);    
					flag_parameters = false;
					base_time = ros::Time::now().toSec();

					waypoints_info.sector.edge_size = range;
					waypoints_info.sector.central_point[0] = central_point[0];
					waypoints_info.sector.central_point[1] = central_point[1];
				}
			}

			else{
				if (base_time+flight_time > ros::Time::now().toSec()){
					waypoints_info.mission_time = base_time+flight_time-ros::Time::now().toSec();
					rho = sqrt(pow(quadrant[index][0]-Drone_info[0],2)+pow(quadrant[index][1]-Drone_info[1],2));
					if (rho < minimum_radius){
						if (index<3){
							index++;
						}
						else{
							index = 0;
						}
					}

					if (index != ant_index){
						waypoint_publish.position.x = quadrant[index][0];
						waypoint_publish.position.y = quadrant[index][1];
						waypoints_pub_.publish(waypoint_publish);
						ant_index = index;
					}	
				}
				else{
					waypoint_publish.position.x = final_pose[0];
					waypoint_publish.position.y = final_pose[1];
					waypoints_pub_.publish(waypoint_publish);
					flag_parameters = true;
					nhp_.setParam("flight_time", 0.0);
					base_time = 0.0;
					waypoints_info.mission_time = base_time;
				}
			}
		}

		fill_waypoints_info(emergency, stop_mission, final_pose, emergency_pose, rho, minimum_radius, flight_time, waypoint_publish);
		waypoint_info_pub_.publish(waypoints_info);
		
		//std::cout <<index<<std::endl;
	   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
	    	rate.sleep();
	}

 return 0;
}
