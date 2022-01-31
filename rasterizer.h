#pragma once

#include "model.h"
#include "vertex.h"

class Rasterizer
{
public:
	Rasterizer(const Light &l, const Eigen::Vector3f &e);
	
	// Bresenham 算法画直线。
	void BHLine(const Eigen::Vector4f &point0, const Eigen::Vector4f &point1);
	// 返回重心坐标 a、b、c 三个值，与向量无关。 
	Eigen::Vector3f BarycentricCoor(const float &x, const float &y, const Vertex* v);
	// 通过重心坐标计算三角形内一点的插值。
	Eigen::Vector4f Interpolate(const float &a, const float &b, const float &c, const Eigen::Vector4f &v1, const Eigen::Vector4f &v2, const Eigen::Vector4f &v3);
	Eigen::Vector3f Interpolate(const float &a, const float &b, const float &c, const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const Eigen::Vector3f &v3);
	Eigen::Vector2f Interpolate(const float &a, const float &b, const float &c, const Eigen::Vector2f &v1, const Eigen::Vector2f &v2, const Eigen::Vector2f &v3);
	float Interpolate(const float &a, const float &b, const float &c, const float& v1, const float& v2, const float& v3);
	// 扫描线光栅化三角形。
	void RasterizeTriangle_SL(Vertex *vertex, Model *model, float *z_bufer);
	// 正交投影并拉伸至屏幕大小，不处理 z 轴。
	void WorldToScreen(Eigen::Vector4f &v);

private:
	Light light;
	Eigen::Vector3f eyePos;
};
