//
//  main.cpp
//  RANSAC
//
//  Created by tomiya on 2017/11/04.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include <iostream>
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include "draw.hpp"
#include "calc.hpp"

using namespace std;

double fov;
double aspect;

int cx, cy; //ドラッグ開始位置
Eigen::Quaterniond cq = Eigen::Quaterniond::Identity();
Eigen::Quaterniond tq;
Eigen::MatrixXd rt;
Eigen::Matrix4d m;

const float radius = 2.0f;
const int n_inliers = 300;
const int n_outliers = 200;
const float noise_scale = 0.2f;
Eigen::Vector3f center(0.f, 0.f, 0.f);
Eigen::Vector3f normal(2.f, 0.f, 2.f);
Eigen::MatrixXf points(3, n_inliers+n_outliers);

void update_points(){
    random_device seed_gen;
    mt19937 engine(seed_gen());
    uniform_real_distribution<> rnd(-1.0, 1.0);
    
    Eigen::Vector3f origin = center;
    Eigen::Vector3f z_axis = normal;
    z_axis.normalize();
    Eigen::Vector3f x_axis;
    if(z_axis(1) != 0.0f) x_axis << 1.0f, -z_axis(0)/z_axis(1), 0.0f;
    else x_axis << 0.0f, 1.0f, 0.0f;
    x_axis.normalize();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    z_axis.normalize();
    
    for(int i = 0; i < n_inliers; i++){
        float x, y, z;
        x = cosf(i * 2 * M_PI / n_inliers) * radius + rnd(engine) * noise_scale;
        y = sinf(i * 2 * M_PI / n_inliers) * radius + rnd(engine) * noise_scale;
        z = 0.0f + rnd(engine) * noise_scale;
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
    fov = 30.0;
    aspect = 1.0;
    m <<
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 10.0,
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
    
    float pre_cx, pre_cy, pre_r;
    ransac_circle_param(points, pre_cx, pre_cy, pre_r, 100, 5.0, 10);
    draw_xyz_axis(2.f);
    draw_circle(center, normal, radius, 64);
    draw_points(points, 3.0f);

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
#if LINE
    glutDisplayFunc(disp_line);
#else
    glutDisplayFunc(disp);
#endif
    glutKeyboardFunc(key);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    init();
    glutMainLoop();
    return 0;
}

