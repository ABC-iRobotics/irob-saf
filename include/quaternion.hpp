/*
 * quaternion.hpp
 *
 *  Created on: 2016. okt. 25.
 *      Author: tamas
 */

#ifndef QUATERNION_HPP_
#define QUATERNION_HPP_

#include <iostream>
#include "vector_3d.hpp"
// TODO replace with double w, Vector3D v
class Quaternion {
 
   public:
   	double w;
   	Vector3D v;
   	
   	Quaternion();
   	Quaternion(double, double, double, double);
   	Quaternion(const Quaternion&);
   	
   	void swap(Quaternion&);
   	Quaternion operator=(const Quaternion&);
   	
   	Quaternion operator+=(const Quaternion&);
   	Quaternion operator-=(const Quaternion&);
   	
   	Quaternion operator/=(const double&);
   	Quaternion operator*=(const double&);
   	
   	Quaternion operator+(const Quaternion&) const;
   	Quaternion operator-(const Quaternion&) const;
   	
   	Quaternion operator*(const double&) const;
   	Quaternion operator/(const double&) const;
   	
   	double length() const;
   	
   	Quaternion normalize();
   	
   
   friend std::ostream& operator<<(std::ostream&, const Quaternion&);
   friend std::istream& operator>>(std::istream&, Quaternion&);
};

#endif

