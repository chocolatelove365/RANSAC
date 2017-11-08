//
//  main.cpp
//  RANSAC
//
//  Created by tomiya on 2017/11/04.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include "json.hpp"
#include "draw.hpp"
#include "calc.hpp"
#include "param.hpp"

using json = nlohmann::json;
using namespace std;

Eigen::MatrixXf model_points, model_out_circle, model_in_circle;

double fov;
double aspect;

int cx, cy; //ドラッグ開始位置
Eigen::Quaterniond cq = Eigen::Quaterniond::Identity();
Eigen::Quaterniond tq;
Eigen::MatrixXd rt;
Eigen::Matrix4d m;

const float radius = 2.0f;
const int n_inliers = 200;
const int n_outliers = 0;
const float noise_scale = 0.05f;
Eigen::Vector3f center(0.f, 0.f, 0.f);
Eigen::Vector3f normal(2.f, 0.f, 2.f);
Eigen::MatrixXf points(3, n_inliers+n_outliers);

bool load_model_json(const char *path){
    std::ifstream input_json(path);
    if(input_json.is_open()){
        json j;
        input_json >> j;
        int m, n;
        std::vector<std::vector<double>> model = j["model"];
        n = (int)model.size();
        if(n == 0) return false;
        m = (int)model[0].size();
        if(m == 0) return false;
        model_points = Eigen::MatrixXf(m, n);
        for(int i = 0; i < n; i++){
            for(int j = 0; j < m; j++){
                model_points(j, i) = model[i][j];
            }
        }
        std::vector<std::vector<double>> model2 = j["model_out_circle"];
        n = (int)model2.size();
        if(n == 0) return false;
        m = (int)model2[0].size();
        if(m == 0) return false;
        model_out_circle = Eigen::MatrixXf(m, n);
        for(int i = 0; i < n; i++){
            for(int j = 0; j < m; j++){
                model_out_circle(j, i) = model2[i][j];
            }
        }
        std::vector<std::vector<double>> model3 = j["model_in_circle"];
        n = (int)model3.size();
        if(n == 0) return false;
        m = (int)model3[0].size();
        if(m == 0) return false;
        model_in_circle = Eigen::MatrixXf(m, n);
        for(int i = 0; i < n; i++){
            for(int j = 0; j < m; j++){
                model_in_circle(j, i) = model3[i][j];
            }
        }
        return true;
    }
    else{
        cout << "ERROR: Could not open json file\n";
        return false;
    }
}

void update_points(){
    random_device seed_gen;
    mt19937 engine(seed_gen());
    uniform_real_distribution<> rnd(-1.0, 1.0);
    
    Eigen::Vector3f origin = center;
    Eigen::Vector3f z_axis = normal;
    z_axis.normalize();
    Eigen::Vector3f x_axis;
    x_axis << z_axis(1), -z_axis(0), 0.0f;
    x_axis.normalize();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    z_axis.normalize();
    
    for(int i = 0; i < n_inliers; i++){
        float x, y, z;
        x = cosf(i * 2 * M_PI / n_inliers) * radius + rnd(engine) * noise_scale;
        y = sinf(i * 2 * M_PI / n_inliers) * radius + rnd(engine) * noise_scale;
#if 1
        z = 0.0f + rnd(engine) * noise_scale;
#else
        z = 0.0f;
#endif
        points.col(i) = x * x_axis + y * y_axis + z * z_axis + origin;
    }
    uniform_real_distribution<> rnd2(-5.0, 5.0);
    for(int i = n_inliers; i < n_inliers+n_outliers; i++){
        points(0, i) = rnd2(engine);
        points(1, i) = rnd2(engine);
        points(2, i) = rnd2(engine);
    }
}

void init(){
    load_model_json("model.json");
    fov = 30.0;
    aspect = 1.0;
    m <<
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 20.0,
    0.0, 0.0, 0.0, 1.0;
    update_points();
}

void disp(){
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, aspect, 0.0001, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Eigen::Matrix4d m_inv = m.inverse();
    glMultMatrixd(m_inv.data());
    if(rt.data() != NULL) glMultMatrixd(rt.data());
    CircleParam param1, param2;
//    ransac_circle_param(model_out_circle, param1, 200, 0.05, 10);
//    ransac_circle_param(model_in_circle, param2, 200, 0.1, 10);
    RANSAC<CircleParam> ransac;
    ransac.update_param(model_in_circle, 200, 0.1, 10);
    draw_xyz_axis(2.f);
    glColor4f(1.f, 1.f, 1.f, 1.f);
#if 0
    draw_circle(center, normal, radius, 64);
#endif
#if 0
    draw_points(points, 3.0f);
#endif
    draw_points(model_points, 2.0f);
    glColor4f(0.f, 1.f, 1.f, 1.f);
    draw_points(model_out_circle, 4.0f);
    draw_circle(param1, 64);
    glColor4f(0.f, 0.f, 1.f, 1.f);
    draw_points(model_in_circle, 4.0f);
    draw_circle(ransac.param, 64);
//    draw_circle(param2, 64);
    glutSwapBuffers();
}

void key(unsigned char key, int x, int y){
    switch (key) {
        case ' ':
            update_points();
            glutSwapBuffers();
            break;
        default:
            break;
    }
}

void special_key(int key, int x, int y){
    switch(key){
        case GLUT_KEY_UP:
            m(3, 3) += 0.05;
            glutPostRedisplay();
            break;
        case GLUT_KEY_DOWN:
            m(3, 3) -= 0.05;
            glutPostRedisplay();
            break;
    }
}

void mouse(int button, int state, int x, int y){
    if(button == GLUT_LEFT_BUTTON){
        if(state == GLUT_DOWN){
            cx = x;
            cy = y;
        }
        else if(state == GLUT_UP){
            cq = tq;
        }
    }
}

void motion(int x, int y){
    double dx, dy, a;
    int wh = glutGet(GLUT_WINDOW_HEIGHT);
    dx = (double)(x - cx) / wh;
    dy = (double)(y - cy) / wh;
    a = sqrt(dx * dx + dy * dy);
    if(a != 0){
        double angle = a * M_PI;
        Eigen::Vector3d axis(dy/a, dx/a, 0.0);
        Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
        tq = dq * cq;
        Eigen::Affine3d mat(tq);
        rt = mat.matrix();
    }
    glutPostRedisplay();
}

int main(int argc, char * argv[]) {
    glutInit(&argc, argv);
    glutInitWindowSize(800,800);
    glutInitWindowPosition(0, 100);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("OpenGLTest");
    glutDisplayFunc(disp);
    glutKeyboardFunc(key);
    glutSpecialFunc(special_key);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    init();
    glutMainLoop();
    return 0;
}

