/*
    This file is part of ENPS_DWA.
    
    https://github.com/RGNC/enps_dwa
    ENPS_DWA is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    RENPSM is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with RENPSM.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <ros/ros.h>
#include <vector>
#include <random>
#include <chrono>
#include <fstream>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <lightsfm/sfm.hpp>
#include <lightsfm/rosmap.hpp>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <enps_dwa/simulator.hpp>


namespace enps
{

const double PERSON_MESH_SCALE = (2.0 / 8.5 * 1.8)*0.9;

struct Metrics
{
	double distanceToNearestPerson;
	double distanceToNearestObstacle;
	double velocity;
};

class Node
{
public:
	Node(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Node() {}

private:
	Metrics computeMetrics(unsigned index);
	void publishTrajectories(const ros::Time& current_time);
    bool publishOdom(const ros::Time& current_time);
    void publishScan360(const ros::Time& current_time);
    bool publishPeople(const ros::Time& current_time);
    void publishDetections(const ros::Time& current_time);
    utils::Vector2d extendForce(utils::Vector2d f, double r) ;
    bool transformPose(double& x, double& y, double& theta, 
				const std::string& sourceFrameId, const std::string& targetFrameId) const;	
	bool transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const;

	void initAgents(int max_people);
	int getPersonCollisionIndex(const utils::Vector2d& x, double collisionThreshold) const;
	
	void publishForces(const ros::Time& current_time);
	void publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& x,const utils::Vector2d& force, 
					visualization_msgs::MarkerArray& markers,const ros::Time& current_time) ;

	static std_msgs::ColorRGBA getColor(double r, double g, double b, double a);
	void setGoals(); 
    std::vector<sfm::Agent> agents;
    tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;
	sensor_msgs::LaserScan scan360;	
	ros::Publisher odom_pub;
	ros::Publisher scan360_pub;
	ros::Publisher forces_pub;
	ros::Publisher detection_markers_pub;
	ros::Publisher trajectories_pub;
	std::mt19937 gen;
	double scan_range_max;
	double pose_initial_x,pose_initial_y,pose_initial_yaw,robot_radius,robot_max_velocity;	
	double person_radius, people_average_vel,people_sd_vel,people_detection_range;
	ros::Publisher people_markers_pub;
	visualization_msgs::MarkerArray markers;
	
	// simulator
	simulator::InputVariables input;
	simulator::UserVariables user;
	simulator::SystemVariables system;


};

std_msgs::ColorRGBA Node::getColor(double r, double g, double b, double a)
{
	std_msgs::ColorRGBA color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	return color;
}
Metrics Node::computeMetrics(unsigned index)
{
	utils::Vector2d pos = agents[index].position;
	Metrics metrics;
	metrics.distanceToNearestPerson = 9999999;
	metrics.distanceToNearestObstacle = sfm::MAP.getNearestObstacle(pos).distance;
	metrics.velocity = agents[index].velocity.norm();
	for (unsigned i=0; i < agents.size(); i++) {
		if (i == index) {
			continue;
		}
		double d = (pos - agents[i].position).norm();
		if (d < metrics.distanceToNearestPerson) {
			metrics.distanceToNearestPerson = d;
		}
	}
	return metrics;

}



void Node::initAgents(int max_people)
{
	agents.resize(max_people+1);
	agents[0].position.set(pose_initial_x,pose_initial_y);
	agents[0].yaw = utils::Angle::fromRadian(pose_initial_yaw);
	agents[0].radius = robot_radius;
	agents[0].desiredVelocity = robot_max_velocity;
	agents[0].teleoperated = true;
	
	
	std::normal_distribution<double> randomVel(people_average_vel,people_sd_vel);	

	utils::Vector2d p(5,5);
	for (int i=1; i<=max_people; i++) {
		while (sfm::MAP.getNearestObstacle(p).distance < 0.5) {
			p.setX(p.getX()+3);
			if (p.getX()>18) {
				p.setY(p.getY()+3);
				p.setX(5);
			}
		}
		agents[i].position = p;
		p.setX(p.getX()+3);
		if (p.getX()>18) {
			p.setY(p.getY()+3);
			p.setX(5);
		}
		
		agents[i].desiredVelocity = randomVel(gen);
		agents[i].radius = person_radius;
		
		
	}
}


void Node::setGoals() {
	std::uniform_real_distribution<double> randomX(-2,2);
    std::uniform_real_distribution<double> randomY(-2,2);
    utils::Vector2d g;
	for (unsigned i=0; i< agents.size();i++) {
		if (agents[i].goals.size()!=0) {
			continue;
		}
		g.set(randomX(gen)+agents[i].position.getX(), randomY(gen)+agents[i].position.getY());
		while (sfm::MAP.getNearestObstacle(g).distance < 1) {
			g.set(randomX(gen)+agents[i].position.getX(), randomY(gen)+agents[i].position.getY());
		}
		sfm::Goal goal;
		goal.center = g;
		goal.radius = 0.5;
		agents[i].goals.push_back(goal);
		if (i==0) {
			user.g1 = g.getX();
			user.g2 = g.getY();
			

		}
		
	}
}


bool Node::publishOdom(const ros::Time& current_time)
{

	double x = agents[0].position.getX();
	double y = agents[0].position.getY();
	double yaw = agents[0].yaw.toRadian();
	
	input.r1 = x;
	input.r2 = y;
	input.theta = yaw; 
	input.v1 = agents[0].velocity.getX();
	input.v2 = agents[0].velocity.getY();

	if (!transformPose(x,y,yaw,"map","odom")) {
		return false;
	}
	nav_msgs::Odometry odom;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.header.stamp = current_time;
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
	odom.twist.twist.linear.x = agents[0].linearVelocity;
	odom.twist.twist.linear.y = 0.0; 
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = agents[0].angularVelocity;
	odom_pub.publish(odom);
	
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.z = 0;
	odom_trans.header.stamp = current_time;
	odom_trans.transform.translation.x = odom.pose.pose.position.x;
	odom_trans.transform.translation.y = odom.pose.pose.position.y;
	odom_trans.transform.rotation = odom.pose.pose.orientation;
	tf_broadcaster.sendTransform(odom_trans);
	return true;
}


int Node::getPersonCollisionIndex(const utils::Vector2d& x, double collisionThreshold) const
{
	double d;
	for (unsigned i = 1; i< agents.size(); i++) {
		d = (agents[i].position - x).squaredNorm();
		if (d<=collisionThreshold) {
			return i;
		}
	}
	return -1;
}


void Node::publishScan360(const ros::Time& current_time)
{
	input.o1.clear();
	input.o2.clear();
	agents[0].obstacles.clear();
	utils::Vector2d x,u;
	double angle=agents[0].yaw.toRadian();
	int collisionIndex;
	double collisionThreshold =person_radius * person_radius;
	angle-=M_PI;
	double d;		
	double maxX = sfm::MAP.getInfo().width * sfm::MAP.getInfo().resolution;
	double maxY = sfm::MAP.getInfo().height * sfm::MAP.getInfo().resolution; 
	
	for (unsigned i=0;i<scan360.ranges.size();i++) {
		u.set(std::cos(angle)*sfm::MAP.getInfo().resolution,std::sin(angle)*sfm::MAP.getInfo().resolution);		
		x=agents[0].position;
		d=0;
		collisionIndex=-1;
		while(x.getX()>0 && x.getY()>0 && x.getX()<maxX && x.getY()<maxY && 
			d<scan_range_max && !sfm::MAP.isObstacle(x) && 
			(collisionIndex = getPersonCollisionIndex(x,collisionThreshold))==-1) {
			x+=u;
			d +=sfm::MAP.getInfo().resolution;
		}
		if (collisionIndex!=-1) {
			d = scan_range_max;
		}
		scan360.ranges[i] = std::max(robot_radius,std::min(d,scan_range_max));
		if (scan360.ranges[i]> robot_radius && scan360.ranges[i]<scan_range_max) {
			input.o1.push_back(x.getX());
			input.o2.push_back(x.getY());
			agents[0].obstacles.push_back(x);
		}
		
		angle+=scan360.angle_increment;
	}
	
	scan360.header.stamp = current_time;
	scan360_pub.publish(scan360);	

}
void Node::publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& x,const utils::Vector2d& force, 
					visualization_msgs::MarkerArray& markers,const ros::Time& current_time) 
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = current_time;
	marker.id = index;
	marker.action = 0;
	marker.color = color;
	marker.lifetime = ros::Duration(1.0);
	marker.scale.x = std::max(1e-4,force.norm());
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.position.x = x.getX();
	marker.pose.position.y = x.getY();
	marker.pose.position.z = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,force.angle().toRadian());
	markers.markers.push_back(marker);
}

void Node::publishDetections(const ros::Time& current_time) {
	visualization_msgs::MarkerArray markers;
	input.p1.clear();
	input.p2.clear();
	input.pv1.clear();
	input.pv2.clear();
	double d;
	unsigned counter = 0;
	for (unsigned i = 1; i< agents.size(); i++) {
		d = (agents[i].position - agents[0].position).norm();
		if (d<people_detection_range) {
			input.p1.push_back(agents[i].position.getX());
			input.p2.push_back(agents[i].position.getY());
			input.pv1.push_back(agents[i].velocity.getX());
			input.pv2.push_back(agents[i].velocity.getY());
			
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = current_time;
			marker.id = counter++;
			marker.type = 3;
			marker.lifetime = ros::Duration(0.1);
			marker.color.a = 0.5;
			marker.color.r =1;
			marker.color.g =0;
			marker.color.b =0;
			marker.scale.x = 2*person_radius;
			marker.scale.y = 2*person_radius;
			marker.scale.z = 0.1;
			marker.pose.position.x = agents[i].position.getX();
			marker.pose.position.y = agents[i].position.getY();
			marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
			
			
			markers.markers.push_back(marker);
		}

	}
	detection_markers_pub.publish(markers);
}

utils::Vector2d Node::extendForce(utils::Vector2d f, double r) {
	double n = f.norm();
	if (n>0.1) {
		n+= r;
		f.normalize();
		f*= n;
	}
	return f;
}


void Node::publishTrajectories(const ros::Time& current_time) {
	for (unsigned i=0;i<markers.markers.size();i++) {
		markers.markers[i].header.stamp = current_time;
		markers.markers[i].lifetime = ros::Duration(1.0);
		if (i==system.selectedIndex) {
			markers.markers[i].scale.x = 0.2;
			markers.markers[i].color.r = 1.0;
			markers.markers[i].color.g = 0;
			markers.markers[i].color.b = 0;
		} else {
			markers.markers[i].scale.x = 0.01;
			markers.markers[i].color.r = 0.5;
			markers.markers[i].color.g = 0.5;
			markers.markers[i].color.b = 0.5;
		}
	}
	trajectories_pub.publish(markers);
}

void Node::publishForces(const ros::Time& current_time) {
	visualization_msgs::MarkerArray markers;
	for(unsigned i=0;i<agents.size();i++) {
		double angle = agents[i].yaw.toRadian();
		
		publishForceMarker(i,getColor(0,0,1,1),agents[i].position,extendForce(agents[i].forces.obstacleForce,agents[i].radius),markers,current_time);
		publishForceMarker(i+agents.size(),getColor(0,1,1,1),agents[i].position,extendForce(agents[i].forces.socialForce,agents[i].radius),markers,current_time);	
		publishForceMarker(i+2*agents.size(),getColor(1,1,0,1),agents[i].position,extendForce(agents[i].velocity,agents[i].radius),markers,current_time);	
		//publishForceMarker(i+3*agents.size(),getColor(1,0,0,1),agents[i].position,agents[i].forces.desiredForce,markers,current_time);	
	}
	forces_pub.publish(markers);
}



bool Node::publishPeople(const ros::Time& current_time)
{
	visualization_msgs::MarkerArray marker_array;
	for(unsigned i=0;i<agents.size();i++) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = current_time;
		marker.id = i-1;
		marker.type = 10;
		marker.pose.position.x = agents[i].position.getX();
		marker.pose.position.y = agents[i].position.getY();
		
		marker.action = 0; 
		
		marker.color.a = 1.0;
		
		if (i==0) {
			marker.mesh_resource = "package://enps_dwa/images/full_TERESA.dae";
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.r =0;
			marker.color.g = 0;
			marker.color.b =1;
			marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, agents[i].yaw.toRadian()+M_PI*1.5);
		} else {
			marker.color.r = 1;
			marker.color.g =0;
			marker.color.b =0;
			marker.scale.x = PERSON_MESH_SCALE;
			marker.scale.y = PERSON_MESH_SCALE;
			marker.scale.z = PERSON_MESH_SCALE;
			marker.mesh_resource = "package://enps_dwa/images/animated_walking_man.mesh";
			marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI*0.5, 0.0, agents[i].yaw.toRadian()+M_PI*0.5);	
		}
		
  		
		marker_array.markers.push_back(marker);

		

	}
	people_markers_pub.publish(marker_array);
	return true;
}


bool Node::transformPose(double& x, double& y, double& theta, const std::string& sourceFrameId, const std::string& targetFrameId) const
{
	tf::Stamped<tf::Pose> pose,tfPose;
	pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,theta), tf::Vector3(x,y,0)));
	pose.frame_id_ = sourceFrameId;
	pose.stamp_ = ros::Time(0);
	try
	{
		tf_listener.transformPose(targetFrameId, pose, tfPose);
	} catch(std::exception &e) {
		//ROS_ERROR("%s",e.what());
		return false;
	}
	x = tfPose.getOrigin().getX();
	y = tfPose.getOrigin().getY();
	tf::Matrix3x3 m(tfPose.getRotation());
	double roll,pitch;
	m.getRPY(roll, pitch, theta);
	return true;
}



bool Node::transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const
{
	tf::Stamped<tf::Pose> pose,tfPose;
	pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(x,y,0)));
	pose.frame_id_ = sourceFrameId;
	pose.stamp_ = ros::Time(0);
	try
	{
		tf_listener.transformPose(targetFrameId, pose, tfPose);
	} catch(std::exception &e) {
		//ROS_ERROR("%s",e.what());
		return false;
	}
	x = tfPose.getOrigin().getX();
	y = tfPose.getOrigin().getY();
	return true;
}



Node::Node(ros::NodeHandle& n, ros::NodeHandle& pn) 
 : gen(std::chrono::system_clock::now().time_since_epoch().count()),
   tf_listener(ros::Duration(10)) {

	double freq,dt;
	std::string odom_id,scan360_id;
	ros::Time current_time,prev_time,metrics_time,init_time;
	int max_people;
	int scan360_readings;

	ROS_INFO("Initiating...");

	simulator::initVariables(system);
	std::ofstream metricsFile("metrics.txt");
	metricsFile<<"Time Agent Velocity distanceToNearestPerson distanceToNearestObstacle"<<std::endl;

	unsigned counter = 0;

	for (double i=-0.8;i<=0.8;i+=0.05) {
		for (double j=0; j<=0.6;j+=0.05) {
			user.angVels.push_back(i);
			user.linVels.push_back(j);
			visualization_msgs::Marker marker;
			marker.points.resize(30);
			sfm::Agent dummy(j,i);
			for (unsigned k = 0; k< 30; k++) {
				marker.points[k].x = dummy.position.getX();
				marker.points[k].y = dummy.position.getY();
				marker.points[k].z = 0;
				dummy.move(0.1);
			}
			marker.header.frame_id="base_link";
			marker.id = counter++;
			marker.type = 4;
			marker.action = 0;
			marker.scale.x = 0.01;
			marker.color.a = 1.0;
			marker.color.r = 0.5;
			marker.color.g = 0.5;
			marker.color.b = 0.5;
			marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);	
			markers.markers.push_back(marker);
			
			
		}
	}
	
	user.k1 = 1.0;
	user.k2 = 0.1;

	pn.param<double>("freq",freq,30);
	pn.param<double>("pose_initial_x", pose_initial_x, 10);
    pn.param<double>("pose_initial_y", pose_initial_y, 10);
 	pn.param<double>("pose_initial_yaw", pose_initial_yaw, 0);
 	pn.param<double>("robot_radius",robot_radius,0.3);
 	pn.param<double>("person_radius",person_radius,0.35);
 	pn.param<double>("robot_max_velocity",robot_max_velocity,1.2);
 	pn.param<std::string>("odom_id",odom_id,"/odom");
 	pn.param<int>("max_people",max_people,20);
 	pn.param<double>("people_average_vel",people_average_vel,1.34);
	pn.param<double>("people_sd_vel",people_sd_vel,0.001);
	pn.param<std::string>("scan360_id",scan360_id,"/scan360");
	pn.param<int>("scan360_readings",scan360_readings,1440);
	pn.param<double>("scan_range_max",scan_range_max,2.5);	
	pn.param<double>("people_detection_range",people_detection_range,2.5);	
 	odom_pub = pn.advertise<nav_msgs::Odometry>(odom_id, 1);
 	detection_markers_pub = pn.advertise<visualization_msgs::MarkerArray>("/detections", 1);
 	trajectories_pub = pn.advertise<visualization_msgs::MarkerArray>("/trajectories", 1);
 	scan360_pub = pn.advertise<sensor_msgs::LaserScan>(scan360_id, 1);
 	people_markers_pub =  pn.advertise<visualization_msgs::MarkerArray>("/people", 1);	
 	forces_pub =  pn.advertise<visualization_msgs::MarkerArray>("/forces", 1);	
 	tf::StampedTransform map_trans(tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,pose_initial_yaw), 
					tf::Vector3(pose_initial_x,pose_initial_y,0.0)), ros::Time::now(), "map", "odom");

	ros::Rate r(freq);
	
	ros::Duration(0.5).sleep();
	sfm::MAP;
	initAgents(max_people);
	sfm::MAP.computeDistances();
	
	scan360.header.frame_id="base_link";
	scan360.angle_min = -M_PI;
	scan360.angle_max = M_PI;
	scan360.angle_increment = 2*M_PI/(double)scan360_readings;	
	scan360.time_increment = 0.0;
	scan360.scan_time = 1.0/freq;
	scan360.range_min = robot_radius;
	scan360.range_max = scan_range_max;
	scan360.ranges.resize(scan360_readings);	
	prev_time = ros::Time::now();
	metrics_time = ros::Time::now();
	init_time = ros::Time::now();
	ROS_INFO("Ok");
	while (n.ok()) {
		current_time = ros::Time::now();
		map_trans.stamp_ = current_time;
		tf_broadcaster.sendTransform(map_trans);	
		dt = (current_time-prev_time).toSec();
		sfm::SFM.updatePosition(agents,dt);
		input.delta = dt;
		prev_time=current_time;
		if ((current_time-metrics_time).toSec()>=1.0) {
			double t = (current_time-init_time).toSec();
			for (unsigned i=0;i<agents.size();i++) {
				Metrics m = computeMetrics(i);
				metricsFile << t <<" "<< i << " "<< m.velocity <<" " << m.distanceToNearestPerson << " " << m.distanceToNearestObstacle << std::endl;
			}
			metrics_time = current_time;
		}
		publishOdom(current_time);
		publishScan360(current_time);
		publishPeople(current_time);
		publishDetections(current_time);
		publishTrajectories(current_time);
		setGoals();
		sfm::SFM.computeForces(agents,&sfm::MAP);
		simulator::run(input,user,system);

		agents[0].linearVelocity = system.selectedLinVel;
		agents[0].angularVelocity = system.selectedAngVel;

 		publishForces(current_time);
		sfm::MAP.computeDistances();
		r.sleep();
		ros::spinOnce();	
	}
	metricsFile.close();

}

}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "enps");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	enps::Node node(n,pn);
	return 0;
}
