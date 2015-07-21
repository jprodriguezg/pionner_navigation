#include <iostream>
#include <string>
#include <vector>
#include <math.h> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <robot_cooperation_project_msgs/waypoints_info.h>
#include <iris_test/GoToWaypoint.h>

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

void fill_waypoints_info(bool emergency, bool stop_mission, std::vector<double> final_pose, std::vector<double> emergency_pose, double rho, double delta_time, iris_test::GoToWaypoint iris_waypoints_service){
	
	waypoints_info.emergency_status = emergency;
	waypoints_info.stop_mission_status = stop_mission;
	waypoints_info.target_point.x = iris_waypoints_service.request.x;
	waypoints_info.target_point.y = iris_waypoints_service.request.y;
	waypoints_info.target_point.z = 1.0;
	waypoints_info.final_position.x = final_pose[0];
	waypoints_info.final_position.y = final_pose[1];
	waypoints_info.final_position.z = 1.0;
	waypoints_info.emergency_position.x = emergency_pose[0];
	waypoints_info.emergency_position.y = emergency_pose[1];
	waypoints_info.emergency_position.z = 1.0;
	waypoints_info.flight_time = delta_time;
	waypoints_info.target_distance = rho;

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
ros::Publisher waypoint_info_pub_=nh_.advertise<robot_cooperation_project_msgs::waypoints_info>("waypoint_info_topic", 1);
ros::ServiceClient iris_waypoints_client =  nh_.serviceClient<iris_test::GoToWaypoint>("gotowaypoint_server");


double rho, base_time = 0.0, delta_time = 0.0, range = 2.0;
std::vector<double> central_point (2,0), final_pose (2,0), emergency_pose(2,0);
bool flag = true, emergency = false, stop_mission = false;
int index = -1, ant_index = -1;

// Define service variables
iris_test::GoToWaypoint iris_waypoints_service;
iris_waypoints_service.request.epsg = 21781;

// Pre definition of quadrant matrix
nhp_.getParam("range",range);
nhp_.getParam("central_point",central_point);
double quadrant[4][2] = {{central_point[0]-range/2, central_point[1]+range/2},
				{central_point[0]+range/2, central_point[1]+range/2},
				{central_point[0]+range/2, central_point[1]-range/2},
				{central_point[0]-range/2, central_point[1]-range/2}};

	while (ros::ok()){

		nhp_.getParam("emergency_position",emergency_pose);
		nhp_.getParam("emergency",emergency);
		nhp_.getParam("stop_mission",stop_mission);

		if(emergency == true || stop_mission == true){
			if(emergency == true){
				iris_waypoints_service.request.x = emergency_pose[0];
				iris_waypoints_service.request.y = emergency_pose[1];
				nhp_.setParam("emergency", false);
			}
			else{
				iris_waypoints_service.request.x = Drone_info[0];
				iris_waypoints_service.request.y = Drone_info[1];
				nhp_.setParam("stop_mission", false);
			
			}
			iris_waypoints_client.call(iris_waypoints_service);
			flag = true;
			nhp_.setParam("delta_time", 0.0);
			delta_time = 0.0;
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

					if (index != ant_index){
						iris_waypoints_service.request.x = quadrant[index][0];
						iris_waypoints_service.request.y = quadrant[index][1];
						iris_waypoints_client.call(iris_waypoints_service);
						ant_index = index;
					}	
				}
				else{
					iris_waypoints_service.request.x = final_pose[0];
					iris_waypoints_service.request.y = final_pose[1];
					iris_waypoints_client.call(iris_waypoints_service);
					flag = true;
					nhp_.setParam("delta_time", 0.0);
					base_time = 0.0;
				}
			}
		}

		fill_waypoints_info(emergency, stop_mission, final_pose, emergency_pose, rho, delta_time, iris_waypoints_service);
		waypoint_info_pub_.publish(waypoints_info);
		
		//std::cout <<delta_time<<std::endl;

		
	   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
	    	rate.sleep();
	}

 return 0;
}
