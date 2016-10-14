/*
 * point_3d.hpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#ifndef POINT_3D_HPP_
#define POINT_3D_HPP_

#include <iostream>

class Point3D {
 
   public:
   	double x;
   	double y;
   	double z;
   	
   	Point3D();
   	Point3D(double, double, double);
   	Point3D(const Point3D&);
   	
   	void swap(Point3D&);
   	Point3D operator=(const Point3D&);
   	
   	Point3D operator+=(const Point3D&);
   	Point3D operator-=(const Point3D&);
   
};


#endif
