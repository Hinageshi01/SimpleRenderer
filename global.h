#pragma once

#include <iostream>
#include <algorithm>
#include <graphics.h>
#include <conio.h>
#include <omp.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <string.h>
#include <time.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>

const double MY_PI = acos(-1);
const int WIDTH = 600;
const int HEIGHT = 600;

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
