#pragma once

#include <iostream>
#include <eigen3/Eigen/Eigen>

const float MY_PI = acos(-1);
const int WIDTH = 512;
const int HEIGHT = 512;

struct Light
{
	Eigen::Vector3f direction;
	Eigen::Vector3f position;
	Eigen::Vector3f intensity;
};

struct Frustum
{
	float fov;
	float aspect;
	float zNear;
	float zFar;
};
