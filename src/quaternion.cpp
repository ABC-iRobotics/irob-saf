/*
 * quaternion.cpp
 *
 *  Created on: 2016. okt. 25.
 *      Author: tamas
 */

#include <iostream>
#include <math.h>
#include "quaternion.hpp"

	Quaternion::Quaternion(): x(0.0), y(0.0), z(0.0), w(0.0) {}

	Quaternion::Quaternion(double x, double y, double z, double w): 
		x(x), y(y), z(z), w(w) {}
	
	Quaternion::Quaternion(const Quaternion& other): 
		x(other.x), y(other.y), z	(other.z), w(other.w){}
   	
   	void Quaternion::swap(Quaternion& other) 
   	{
   		Quaternion tmp(*this);
   		x = other.x;
   		y = other.y;
   		z = other.z;
   		w = other.w;
   		
   		other.x = tmp.x;
   		other.y = tmp.y;
   		other.z = tmp.z;
   		other.w = tmp.w;
   	}
   	
   	Quaternion Quaternion::operator=(const Quaternion& other)
   	{
   		Quaternion tmp(other);
   		this->swap(tmp);
   		return *this;
   	}
   	
   	std::ostream& operator<<(std::ostream& os, const Quaternion& q)
   	{
   		return os << "(" << q.x <<", " << q.y << ", " << q.z << "," << q.w << ")";
   	}

