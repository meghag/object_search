
#include <cmath>
#include <iostream>

#include <../include/van_der_corput.h>

//van der corput sequence
double VanDerCorput::vdc(int n, double base)
{
    double vdc = 0, denom = 1;
    while (n)
    {
        vdc += fmod(n, base) / (denom *= base);
        n /= base; // note: conversion from 'double' to 'int'
    }
    return vdc;
}

int vdc_bases[] = {2,3,5,7,11,13,17,19,23,29};

//get the nth element of a dense sequence of poses filling 0..1 in x, y, z and the SO(3) for rotation
// based on van der corput sequence with relatively prime bases
tf::Pose VanDerCorput::vdc_pose(int n)
{
    tf::Pose ret;
    ret.setOrigin(tf::Vector3(vdc(n,vdc_bases[0]),vdc(n,vdc_bases[1]),vdc(n,vdc_bases[2])));
    double u[3];
    for (int i= 0; i<3; i++)
        u[i] = vdc(n,vdc_bases[i+3]);
    double q[4];
    q[0] = sqrt(1 - u[0]) * sin(2 * M_PI * u[1]);
    q[1] = sqrt(1 - u[0]) * cos(2 * M_PI * u[1]);
    q[2] = sqrt(u[0]) * sin(2 * M_PI * u[2]);
    q[3] = sqrt(u[0]) * cos(2 * M_PI * u[2]);
    ret.setRotation(tf::Quaternion(q[0],q[1],q[2],q[3]));
    return ret;
}

tf::Pose VanDerCorput::vdc_pose_bound(tf::Vector3 min, tf::Vector3 max, int n)
{
    tf::Pose ret = vdc_pose(n);
    ret.getOrigin() = tf::Vector3( min.x() + (ret.getOrigin().x() * (max.x() - min.x())),
                                   min.y() + (ret.getOrigin().y() * (max.y() - min.y())),
                                   min.z() + (ret.getOrigin().z() * (max.z() - min.z())));
    return ret;
}

