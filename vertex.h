#pragma once

#include "global.h"

class Vertex
{
public:
	Eigen::Vector4f pos;
	Eigen::Vector4f normal;
	Eigen::Vector2f uv;

	// viewSpace ����ϵ�µĶ������ꡣ
	Eigen::Vector3f viewPos;
};
