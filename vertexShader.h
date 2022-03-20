#pragma once

#include "global.h"

#undef near
#undef far

class Vertex
{
public:
	Eigen::Vector4f pos;
	Eigen::Vector4f normal;
	Eigen::Vector2f uv;

	// viewSpace 坐标系下的顶点坐标。
	Eigen::Vector3f viewPos;
};

class VertexShader
{
public:
	VertexShader(const float &a, const float &s, const Eigen::Vector3f &m, const Eigen::Vector3f &e, const Frustum &f);

	// 在主循环中被调用，随着键盘输入更新模型的位置信息。
	void Update(const float &a, const float &s);
	// 计算 model 矩阵。
	inline Eigen::Matrix4f GetModelMatrix();
	// 变换至屏幕空间坐标系。
	void Transform(Vertex *v);

private:
	// model 矩阵所需要的数据。
	float angleY;
	float scale;
	Eigen::Vector3f move;

	// 平截头体。
	Frustum fru;

	// 这三个矩阵不出意外是不会改变的，所以偷个懒只在构造函数里计算一次。
	Eigen::Matrix4f view;
	Eigen::Matrix4f projection;
	Eigen::Matrix4f Mmove; // 这个是 model 矩阵中的移动矩阵
};
