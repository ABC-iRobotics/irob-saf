/*
 * quaternion.hpp
 *
 *  Created on: 2016. okt. 25.
 *      Author: tamas
 */

#ifndef QUATERNION_HPP_
#define QUATERNION_HPP_

#include <iostream>

class Quaternion {
 
   public:
   	double x;
   	double y;
   	double z;
   	double w;
   	
   	Quaternion();
   	Quaternion(double, double, double, double);
   	Quaternion(const Quaternion&);
   	
   	void swap(Quaternion&);
   	Quaternion operator=(const Quaternion&);
   	
   	/*Quaternion operator+=(const Quaternion&);
   	Quaternion operator-=(const Quaternion&);
   	
   	Quaternion operator/=(const double&);
   	Quaternion operator*=(const double&);
   	
   	Quaternion operator+(const Quaternion&) const;
   	Quaternion operator-(const Quaternion&) const;
   	
   	Quaternion operator*(const double&) const;
   	Quaternion operator/(const double&) const;*/
   	
   
   friend std::ostream& operator<<(std::ostream&, const Quaternion&);
   friend std::istream& operator>>(std::istream&, Quaternion&);
};

#endif

