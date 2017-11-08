//
//  calc.hpp
//  RANSAC
//
//  Created by tomiya on 2017/11/05.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#ifndef calc_hpp
#define calc_hpp

#include <stdio.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <random>
#include <float.h>
#include "param.hpp"

void ransac_circle_param(Eigen::Matrix<float, 3, Eigen::Dynamic> points, CircleParam &param, int max_loop, float threshold, int min_inliers);

#endif /* calc_hpp */
