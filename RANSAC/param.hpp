//
//  param.hpp
//  RANSAC
//
//  Created by tomiya on 2017/11/08.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#ifndef param_h
#define param_h

#include <Eigen/Core>
#include <Eigen/Geometry>

struct Circle{
    Eigen::Vector3f center;
    Eigen::Vector3f normal;
    float radius;
};

struct Line{
    Eigen::Vector3f p1, p2;
};

#endif /* param_h */
