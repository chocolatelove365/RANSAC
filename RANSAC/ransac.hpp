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

template <typename T>
class RANSAC{
    int n_samples;
    float get_error(Eigen::Vector3f point, T obj);
    void calc_param(Eigen::Matrix<float, 2, Eigen::Dynamic> points, float &cx, float &cy, float &r);
    void calc_with_samples(Eigen::Matrix<float, 3, Eigen::Dynamic> points, T &obj);
public:
    RANSAC();
    void calc(Eigen::Matrix<float, 3, Eigen::Dynamic> points, T &param, int max_loop, float threshold, int min_inliers);
};

#endif /* calc_hpp */
