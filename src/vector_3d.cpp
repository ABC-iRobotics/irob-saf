/*
 * vector_3d.cpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#include <iostream>
#include <math.h>
#include "vector_3d.hpp"

 	Vector3D::Vector3D(): x(0.0), y(0.0), z(0.0) {}
   	Vector3D::Vector3D(double x, double y, double z): x(x), y(y), z(z) {}
   	Vector3D::Vector3D(const Vector3D& other): x(other.x), y(other.y), z(other.z) {}
   	
   	void Vector3D::swap(Vector3D& other) 
   	{
   		Vector3D tmp(*this);
   		x = other.x;
   		y = other.y;
   		z = other.z;
   		
   		other.x = tmp.x;
   		other.y = tmp.y;
   		other.z = tmp.z;
   	}
   	
   	Vector3D Vector3D::operator=(const Vector3D& other)
   	{
   		Vector3D tmp(other);
   		this->swap(tmp);
   		return *this;
   	}
   	
   	Vector3D Vector3D::operator+=(const Vector3D& other) 
   	{
   		x += other.x;
   		y += other.y;
   		z += other.z;
   		
   		return *this;
   	}
   	
   	Vector3D Vector3D::operator-=(const Vector3D& other) 
   	{
   		x -= other.x;
   		y -= other.y;
   		z -= other.z;
   		
   		return *this;
   	}
   	
   	Vector3D Vector3D::operator/=(const double& d) 
   	{
   		x /= d;
   		y /= d;
   		z /= d;
   		
   		return *this;
   	}
   	
   	Vector3D Vector3D::operator*=(const double& d) 
   	{
   		x *= d;
   		y *= d;
   		z *= d;
   		
   		return *this;
   	}
   	
   	Vector3D Vector3D::operator+(const Vector3D& other) const
   	{
   		Vector3D tmp(*this);
   		tmp += other;
   		return tmp;
   	}
   	
   	Vector3D Vector3D::operator-(const Vector3D& other) const
   	{
   		Vector3D tmp(*this);
   		tmp -= other;
   		return tmp;
   	}
   	
   	Vector3D Vector3D::operator*(const double& d) const
   	{
   		Vector3D tmp(*this);
   		tmp *= d;
   		return tmp;
   	}
   	
   	Vector3D Vector3D::operator/(const double& d) const
   	{
   		Vector3D tmp(*this);
   		tmp /= d;
   		return tmp;
   	}
   	
   	double Vector3D::length() const
   	{
   		return sqrt((x*x)+(y*y)+(z*z));
   	}
   	
   	double Vector3D::distance(const Vector3D& other) const
   	{
   		Vector3D d = *this - other;
   		return sqrt((d.x*d.x)+(d.y*d.y)+(d.z*d.z));
   	}
   	
   	std::ostream& operator<<(std::ostream& os, const Vector3D& p)
   	{
   		return os << "(" << p.x <<", " << p.y << ", " << p.z << ")";
   	}


