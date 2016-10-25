/*
 * trajectory.hpp
 *
 *  Created on: 2016. okt. 24.
 *      Author: tamas
 */

#ifndef TRAJECTORY_3D_HPP_
#define TRAJECTORY_3D_HPP_

#include <iostream>
#include "vector_3d.hpp"

class Trajectory {
 
   public:

   
   friend std::ostream& operator<<(std::ostream&, const Trajectory&);
};


#endif
