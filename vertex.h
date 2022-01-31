#pragma once

#include "global.h"

class Vertex
{
public:
	Eigen::Vector4f pos;
	Eigen::Vector4f normal;
	Eigen::Vector2f uv;

	// viewSpace 坐标系下的顶点坐标。
	Eigen::Vector3f viewPos;
};
