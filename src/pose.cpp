/*
 * pose.cpp
 *
 *  Created on: 2016. okt. 27.
 *      Author: tamas
 */

#include <iostream>
#include <math.h>
#include "pose.hpp"

 	Pose::Pose(): position(0.0, 0.0, 0.0), 
 			orientation(0.0, 0.0, 0.0, 0.0), jaw(0.0) {}
 			
   	Pose::Pose(double px, double py, double pz, 
   		double ox, double oy, double oz,double ow,
   		double jaw): 
   			position(px, py, pz),
   			orientation(ox, oy, oz, ow), jaw(jaw) {}
   			
   	Pose::Pose(const Pose& other): position(other.position), 
   		orientation(other.orientation), jaw(other.jaw) {}
   		
	Pose::Pose(const geometry_msgs::PoseStamped& msg, double jaw): 
		position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z), 			orientation(msg.pose.orientation.x, msg.pose.orientation.y,
				 msg.pose.orientation.z, msg.pose.orientation.w), 
		jaw(jaw){}	

   	void Pose::swap(Pose& other) 
   	{
   		Pose tmp(*this);
   		position.x = other.position.x;
   		position.y = other.position.y;
   		position.z = other.position.z;
   		
   		orientation.x = other.orientation.x;
   		orientation.y = other.orientation.y;
   		orientation.z = other.orientation.z;
   		orientation.w = other.orientation.w;
   		
   		jaw = other.jaw;
   		
   		
   		other.position.x = tmp.position.x;
   		other.position.y = tmp.position.y;
   		other.position.z = tmp.position.z;
   		
   		other.orientation.x = tmp.orientation.x;
   		other.orientation.y = tmp.orientation.y;
   		other.orientation.z = tmp.orientation.z;
   		other.orientation.w = tmp.orientation.w;
   		
   		other.jaw = tmp.jaw;
   	}
   	
   	Pose Pose::operator=(const Pose& other)
   	{
   		Pose tmp(other);
   		this->swap(tmp);
   		return *this;
   	}
   	
   	Pose Pose::operator+=(const Vector3D& v) 
   	{
   		position += v;
   		return *this;
   	}
   	
   	Pose Pose::operator-=(const Vector3D& v) 
   	{
   		position -= v;   		
   		return *this;
   	}
   	
   	Pose Pose::operator+(const Vector3D& v) const
   	{
   		Pose tmp(*this);
   		tmp += v;
   		return tmp;
   	}
   	
   	Pose Pose::operator-(const Vector3D& v) const
   	{
   		Pose tmp(*this);
   		tmp -= v;
   		return tmp;
   	}
   	
   	std::ostream& operator<<(std::ostream& os, const Pose& p)
   	{
   		return os << p.position <<"\t" << p.orientation << "\t" << p.jaw;
   	}
   	
   	std::istream& operator>>(std::istream& is, Pose& p)
   	{
   		is >> p.position >> std::ws >> p.orientation >> std::ws 
   			>> p.jaw >> std::ws;
   		return is;
   	}
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	
   	


