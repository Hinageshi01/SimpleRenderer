#pragma once

#include <eigen3/Eigen/Eigen>

constexpr float MY_PI = 3.1415926535897932384626433832795;
constexpr int WIDTH = 512;
constexpr int HEIGHT = 512;
constexpr float Z_MAX = FLT_MAX;

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
