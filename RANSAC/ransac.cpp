//
//  calc.cpp
//  RANSAC
//
//  Created by tomiya on 2017/11/05.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include "ransac.hpp"

using namespace std;

/*********** COMMON **********/
vector<int> get_ramdom_values(int num, int min, int max){
    random_device seed_gen;
    mt19937 engine(seed_gen());
    vector<int> tmp(max-min+1);
    for(int i = 0; i < tmp.size(); i++) tmp[i] = i + min;
    shuffle(tmp.begin(), tmp.end(), engine);
    vector<int> result;
    for(int i = 0; i < num; i++) result.push_back(tmp[i]);
    return result;
}

template <typename T>
void RANSAC<T>::calc(Eigen::Matrix<float, 3, Eigen::Dynamic> points, T &param, int max_loop, float threshold, int min_inliers){
    int n_points = (int)points.cols();
    if(n_points < n_samples){
        cout << "ERROR: At least " << n_samples << " points are needed to calculate the parameter of circle.\n";
        return;
    }
    vector<float> good_error;
    vector<T> good_param;
    for(int i = 0; i < max_loop; i++){
        Eigen::MatrixXf samples(3, n_samples);
        T sample_param;
        vector<int> id = get_ramdom_values(n_samples, 0, n_points-1);
        for(int j = 0; j < n_samples; j++) samples.col(j) = points.col(id[j]);
        calc_with_samples(samples, sample_param);
        Eigen::Matrix<float, 3, Eigen::Dynamic> inliers;
        
        for(int j = 0; j < n_points; j++){
            float error = get_error(points.col(j), sample_param);
            if(error > threshold) continue;
            else{
                inliers.conservativeResize(inliers.rows(), inliers.cols()+1);
                inliers.col(inliers.cols()-1) = points.col(j);
            }
        }
        if(inliers.cols() > min_inliers){
            float current_error = 0;
            for(int j = 0; j < n_points; j++) current_error += get_error(points.col(j), sample_param);
            current_error /= n_points;
            good_param.push_back(sample_param);
            good_error.push_back(current_error);
        }
    }
    
    if(good_error.size() > 0){
        int best_index = 0;
        float best_error = FLT_MAX;
        for(int i = 0; i < good_error.size(); i++){
            if(good_error[i] < best_error){
                best_error = good_error[i];
                best_index = i;
            }
        }
        param = good_param[best_index];
    }
}

/*********** CIRCLE **********/
template <>
RANSAC<Circle>::RANSAC(){
    n_samples = 3;
}

template <>
float RANSAC<Circle>::get_error(Eigen::Vector3f point, Circle param){
    Eigen::Vector3f v1 = point - param.center;
    Eigen::Vector3f v2 = param.normal.cross(v1.cross(param.normal));
    return (v1-v2).norm();
}

template <>
void RANSAC<Circle>::calc_param(Eigen::Matrix<float, 2, Eigen::Dynamic> points, float &cx, float &cy, float &r){
    if(points.cols() < 3){
        cout << "ERROR: At least 3 points are needed to calculate the parameter of circle.\n";
        return;
    }
    Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    Eigen::Vector3f x;
    for(int i = 0; i < points.cols(); i++){
        M(0,0) += points(0,i)*points(0,i);
        M(0,1) += points(0,i)*points(1,i);
        M(0,2) += points(0,i);
        M(1,0) += points(0,i)*points(1,i);
        M(1,1) += points(1,i)*points(1,i);
        M(1,2) += points(1,i);
        M(2,0) += points(0,i);
        M(2,1) += points(1,i);
        M(2,2) += 1;
        v(0) -= points(0,i)*points(0,i)*points(0,i)+points(0,i)*points(1,i)*points(1,i);
        v(1) -= points(0,i)*points(0,i)*points(1,i)+points(1,i)*points(1,i)*points(1,i);
        v(2) -= points(0,i)*points(0,i)+points(1,i)*points(1,i);
    }
    x = M.inverse() * v;
    cx = -x(0)/2;
    cy = -x(1)/2;
    r = sqrt(cx*cx+cy*cy-x(2));
}

template <>
void RANSAC<Circle>::calc_with_samples(Eigen::Matrix<float, 3, Eigen::Dynamic> points, Circle &param){
    if(points.cols() !=  3 || n_samples != 3){
        cout << "ERROR: Just " << n_samples <<  " sample points are needed.\n";
        return;
    }
    Eigen::Matrix3f PT = points.transpose();
    Eigen::Matrix3f PT_inv = PT.inverse();
    Eigen::Vector3f z_axis = PT_inv * Eigen::Vector3f::Ones();
    Eigen::Vector3f x_axis(z_axis(1), -z_axis(0), 0.0f);
    z_axis.normalize();
    x_axis.normalize();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    Eigen::Matrix3f R;
    R << x_axis, y_axis, z_axis;
    Eigen::Matrix3f _points = R.inverse() * points;
    cout << "points: \n" << _points << "\n";
    float _cx, _cy, _cz, _r;
    Eigen::Matrix<float, 2, Eigen::Dynamic> _points2d = _points.topRows(2);
    _cz = _points(2, 0);
    calc_param(_points2d, _cx, _cy, _r);
    Eigen::Vector3f _center(_cx, _cy, _cz);
    param.center = R * _center;
    param.normal = z_axis;
    param.radius = _r;
}

/*********** LINE **********/
template <>
RANSAC<Line>::RANSAC(){
    n_samples = 2;
}

template <>
float RANSAC<Line>::get_error(Eigen::Vector3f point, Line param){
    Eigen::Vector3f pos = param.position;
    Eigen::Vector3f dir = param.direction;
    float t = (point-pos).dot(dir) / dir.dot(dir);
    return (point-(pos+t*dir)).norm();
}

template <>
void RANSAC<Line>::calc_with_samples(Eigen::Matrix<float, 3, Eigen::Dynamic> points, Line &param){
    if(points.cols() != 2 || n_samples != 2){
        cout << "ERROR: Just " << n_samples <<  " sample points are needed.\n";
        return;
    }
    param.position = points.col(0);
    param.direction = (points.col(1) - points.col(0)).normalized();
//    cout << "direction: \n" << param.direction << "\n";
}

template class RANSAC<Circle>;
template class RANSAC<Line>;
