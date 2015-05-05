#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <pionner_project_messages/gps_extension.h>

#define PI 3.14159265

float longitude,latitude,altitude;

void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){

	longitude = msg ->longitude;
	latitude = msg ->latitude;
	altitude = msg ->altitude;

  return;
}

int main(int argc, char** argv){

ros::init(argc, argv, "joystick2piooner_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

pionner_project_messages::gps_extension publish_data;
bool South_Hemisphere = false;
nh_.getParam("/gps_out_node/South_Hemisphere",South_Hemisphere);

ros::Subscriber joy_sub_=nh_.subscribe("GPS_topic", 10, GPSCallback);
ros::Publisher gps_extension_pub_=nh_.advertise<pionner_project_messages::gps_extension>("gps_extension_topic", 1);

	while(ros::ok()){
	
	/*Semieje mayor (a) - Semieje menor (b) */
	double a=6378388.0;
	double b=6356911.946139;
	
	/* Almaceno la latitud y longitud actuales */
	double lon=longitude;
	double lat=latitude;
	
	/* Latitud y longitud (En radianes) */
	double latRad=(lat*PI)/180;
	double lonRad=(lon*PI)/180;

	/* Radio de curvatura polar (c) - Aplanamiento (d) */
	double c=pow(a,2)/b;
	double d=(a-b)/a;
	
	/* Excentricidad y Segunda Excentricidad */
	double e=sqrt(pow(a,2)-pow(b,2))/a;
	double se=sqrt(pow(a,2)-pow(b,2))/b;

	/* Huso Horario */
	int huso=int((lon/6)+31);

	/*Distancia angular entre la longitud y el meridiano central del uso meridiano central (mc) */
	float mc=lonRad-(((huso*6-183)*PI)/180);

	/* Calculo Parametros */
	double A=cos(latRad)*sin(mc);
	double ep=((double)1/(double)2)*log((1+A)/(1-A));
	double n=atan(tan(latRad)/cos(mc))-latRad;
	double v=(c/sqrt(1+pow(se,2)*pow(cos(latRad),2)))*(float)0.9996;
	double z=(pow(se,2)/2)*pow(ep,2)*pow(cos(latRad),2);
	double A1=sin(2*latRad);
	double A2=A1*pow(cos(latRad),2);
	double J2=latRad+(A1/2);
	double J4=(3*J2+A2)/4;
	double J6=(5*J4+A2*pow(cos(latRad),2))/(float)3;
	double se2=pow(se,2);
	double alpha=((float)3/(float)4)*se2;
	double betha=((float)5/(float)3)*pow((double)alpha,2);
	double gama=((float)35/(float)27)*pow(alpha,3);

	double B=(float)0.9996*c*(latRad-alpha*J2+betha*J4-gama*J6);

	/* Calculo de X y Y*/
	double X=ep*v*(1+(z/3))+(float)500000;
	double Y=n*v*(1+z)+B;

	if(South_Hemisphere == true)
		Y=Y+10000000;

	//std::cout << "X " << X << " Y  " << Y << " Huso Horario " <<huso<<std::endl;	

	publish_data.X = X;
	publish_data.Y = Y;
	gps_extension_pub_.publish(publish_data);

	
		
	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
	} 
  
return 0;
}
