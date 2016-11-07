/*
 * quaternion.cpp
 *
 *  Created on: 2016. okt. 25.
 *      Author: tamas
 */

#include <iostream>
#include <math.h>
#include "quaternion.hpp"

	Quaternion::Quaternion(): w(0.0), v(0.0, 0.0, 0.0) {}

	Quaternion::Quaternion(double x, double y, double z, double w): 
		w(w), v(x,y,z) {}
	
	Quaternion::Quaternion(const Quaternion& other): 
		w(other.w), v(other.v){}
   	
   	void Quaternion::swap(Quaternion& other) 
   	{
   		Quaternion tmp(*this);
   		v.swap(other.v);
   		w = other.w;
   		
   		other.swap(tmp);
   		other.w = tmp.w;
   	}
   	
   	Quaternion Quaternion::operator=(const Quaternion& other)
   	{
   		Quaternion tmp(other);
   		this->swap(tmp);
   		return *this;
   	}
   	
   	
   		Quaternion Quaternion::operator+=(const Quaternion& other) 
   	{
   		v += other.v;
   		w += other.w;
   		
   		return *this;
   	}
   	
   	Quaternion Quaternion::operator-=(const Quaternion& other) 
   	{
   		v -= other.v;
   		w -= other.w;
   		
   		return *this;
   	}
   	
   	Quaternion Quaternion::operator/=(const double& d) 
   	{
   		v /= d;
   		w /= d;
   		
   		return *this;
   	}
   	
   	Quaternion Quaternion::operator*=(const double& d) 
   	{
   		v *= d;
   		w *= d;
   		
   		return *this;
   	}
   	
   	Quaternion Quaternion::operator+(const Quaternion& other) const
   	{
   		Quaternion tmp(*this);
   		tmp += other;
   		return tmp;
   	}
   	
   	Quaternion Quaternion::operator-(const Quaternion& other) const
   	{
   		Quaternion tmp(*this);
   		tmp -= other;
   		return tmp;
   	}
   	
   	Quaternion Quaternion::operator*(const double& d) const
   	{
   		Quaternion tmp(*this);
   		tmp *= d;
   		return tmp;
   	}
   	
   	Quaternion Quaternion::operator/(const double& d) const
   	{
   		Quaternion tmp(*this);
   		tmp /= d;
   		return tmp;
   	}
   	
   	double Quaternion::length() const
   	{
   		return sqrt((v.x*v.x)+(v.y*v.y)+(v.z*v.z)+(w*w));
   	}
   	
   	Quaternion Quaternion::normalize()
   	{
   		(*this)/=length();
   		return *this;
    }
   	
   	
   	std::ostream& operator<<(std::ostream& os, const Quaternion& q)
   	{
   		return os << q.v <<"\t" << q.w;
   	}
   	
   	std::istream& operator>>(std::istream& is, Quaternion& q)
   	{
   		is >> q.v >> std::ws >> q.w >> std::ws;
   		return is;
   	}

