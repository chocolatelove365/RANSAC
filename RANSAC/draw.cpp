//
//  object.cpp
//  SfM
//
//  Created by tomiya on 2017/10/06.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include "draw.hpp"
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>

using namespace std;

void draw_points(float *vtx, int n_vtx, float size){
    glVertexPointer(3, GL_FLOAT, 0, vtx);
    glPointSize(size);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, n_vtx);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void draw_points(Eigen::Matrix<float, 3, Eigen::Dynamic> vtx, float size){
    draw_points(vtx.data(), (int)vtx.cols(), size);
}

void draw_circle(Eigen::Vector3f center, Eigen::Vector3f normal, float radius, int sides){
    draw_circle(center(0), center(1), center(2), normal(0), normal(1), normal(2), radius, sides);
}

void draw_circle(float cx, float cy, float cz, float nx, float ny, float nz, float radius, int sides){
    if(nx == 0 && ny == 0 && nz == 0){
        cout << "ERROR: Normal is zero vector.\n";
        return;
    }
    Eigen::Vector3f origin(cx , cy, cz);
    Eigen::Vector3f z_axis(nx, ny, nz);
    z_axis.normalize();
    Eigen::Vector3f x_axis;
    x_axis << z_axis(1), -z_axis(0), 0.0f;
    x_axis.normalize();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    z_axis.normalize();
    float vtxs[sides*3];
    for(int i = 0; i < sides; i++){
        float x, y, z;
        x = cosf(i * 2 * M_PI / sides) * radius;
        y = sinf(i * 2 * M_PI / sides) * radius;
        z = 0.0f;
        Eigen::Vector3f vtx = x * x_axis + y * y_axis + z * z_axis + origin;
        vtxs[i*3+0] = vtx(0);
        vtxs[i*3+1] = vtx(1);
        vtxs[i*3+2] = vtx(2);
    }
    glVertexPointer(3, GL_FLOAT, 0, vtxs);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_LINE_LOOP, 0, sides);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void draw_line(float a, float b, float min_x, float max_x){
    float vtx[4];
    vtx[0] = min_x;
    vtx[1] = a * min_x + b;
    vtx[2] = max_x;
    vtx[3] = a * max_x + b;
    glVertexPointer(2, GL_FLOAT, 0, vtx);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_LINES, 0, 2);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void draw_xyz_axis(float line_width){
    static const GLfloat vtx[] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    static const GLfloat color[] = {
        1.0f, 0.0f, 0.0f, 1.0f,
        1.0f, 0.0f, 0.0f, 1.0f,
        0.0f, 1.0f, 0.0f, 1.0f,
        0.0f, 1.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f, 1.0f,
        0.0f, 0.0f, 1.0f, 1.0f
    };
    glVertexPointer(3, GL_FLOAT, 0, vtx);
    glColorPointer(4, GL_FLOAT, 0, color);
    glLineWidth(line_width);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glDrawArrays(GL_LINES, 0, 6);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
}
