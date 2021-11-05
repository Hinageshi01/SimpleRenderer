#pragma once

#include "global.h"

#undef near
#undef far

class VertexShader
{
public:
	VertexShader(const float& a, const float& s, const Eigen::Vector3f& m, const Eigen::Vector3f& e, const Frustum& f);

	// 更新位置信息
	void Update(const float& a, const Eigen::Vector3f& m);
	// model 矩阵。
	inline Eigen::Matrix4f GetModelMatrix();
	// view 矩阵。
	inline Eigen::Matrix4f GetViewMatrix();
	// 变换至屏幕空间坐标系。
	void Transform(Eigen::Vector4f* vertex, Eigen::Vector4f* normal);

private:
	float angleY;
	float scale;

	Eigen::Vector3f move;
	Eigen::Vector3f eyePos;
	Frustum fru;

	float near;
	float far;

	// projection 矩阵不出意外是不会改变的，在构造函数里计算一次即可。
	Eigen::Matrix4f projection;
};
