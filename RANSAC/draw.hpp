//
//  object.hpp
//  SfM
//
//  Created by tomiya on 2017/10/06.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#ifndef object_hpp
#define object_hpp

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "param.hpp"


void draw_points(float *vtx, int n_vtx, float size);
void draw_points(Eigen::Matrix<float, 3, Eigen::Dynamic> vtx, float size);
void draw_circle(Circle param, int sides);
void draw_circle(Eigen::Vector3f center, Eigen::Vector3f normal, float radius, int sides);
void draw_circle(float cx, float cy, float cz, float nx, float ny, float nz, float radius, int sides);
void draw_line(float a, float b, float min_x, float max_x);
void draw_xyz_axis(float line_width=1.0);

#endif /* object_hpp */
