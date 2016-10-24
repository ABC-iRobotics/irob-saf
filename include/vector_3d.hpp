/*
 * vector_3d.hpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#ifndef VECTOR_3D_HPP_
#define VECTOR_3D_HPP_

#include <iostream>

class Vector3D {
 
   public:
   	double x;
   	double y;
   	double z;
   	
   	Vector3D();
   	Vector3D(double, double, double);
   	Vector3D(const Vector3D&);
   	
   	void swap(Vector3D&);
   	Vector3D operator=(const Vector3D&);
   	
   	Vector3D operator+=(const Vector3D&);
   	Vector3D operator-=(const Vector3D&);
   	
   	Vector3D operator/=(const double&);
   	Vector3D operator*=(const double&);
   	
   	Vector3D operator+(const Vector3D&) const;
   	Vector3D operator-(const Vector3D&) const;
   	
   	Vector3D operator*(const double&) const;
   	Vector3D operator/(const double&) const;
   	
   	double distance(const Vector3D&) const;
   
   friend std::ostream& operator<<(std::ostream&, const Vector3D&);
};


#endif
