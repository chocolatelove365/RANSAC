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

struct CircleParam{
    Eigen::Vector3f center;
    Eigen::Vector3f normal;
    float radius;
};

#endif /* param_h */
