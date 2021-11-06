#pragma once

#include "vertex.h"

#undef near
#undef far

class VertexShader
{
public:
	VertexShader(const float& a, const float& s, const Eigen::Vector3f& m, const Eigen::Vector3f& e, const Frustum& f);

	// 更新位置信息。
	void Update(const float& a, const float& s);
	// 计算 model 矩阵。
	inline Eigen::Matrix4f GetModelMatrix();
	// 变换至屏幕空间坐标系。
	void Transform(Vertex* v);

private:
	float angleY;
	float scale;
	Eigen::Vector3f move;

	// 平截头体。
	Frustum fru;

	// 这两个矩阵不出意外是不会改变的，在构造函数里计算一次即可。
	Eigen::Matrix4f view;
	Eigen::Matrix4f projection;
};
