#pragma once

#include "global.h"
#include "model.h"

class Rasterizer
{
public:
	Rasterizer(const Light& l);
	
	// Bresenham 算法画直线。
	void BHLine(const Eigen::Vector4f& point0, const Eigen::Vector4f& point1, COLORREF color);
	// 返回重心坐标 a、b、c 三个值，与向量无关。 
	inline Eigen::Vector3f BarycentricCoor(const float& x, const float& y, const Eigen::Vector4f* v);
	// 通过重心坐标计算三角形内一点的插值。
	inline Eigen::Vector4f Interpolate(const float& a, const float& b, const float& c, const Eigen::Vector4f& v1, const Eigen::Vector4f& v2, const Eigen::Vector4f& v3);
	// 通过叉乘判断一点是否在三角形内。
	inline bool InsideTriangle(const Eigen::Vector4f* v, const Eigen::Vector3f& p);
	// 包围盒光栅化三角形。
	void RasterizeTriangle_AABB(Eigen::Vector4f* v, Eigen::Vector4f* n, float* z_bufer);
	// 扫描线光栅化三角形。
	void RasterizeTriangle_SL(Eigen::Vector4f* v, Eigen::Vector4f* n, float* z_bufer);
	// 正交投影并拉伸至屏幕大小，不处理 z 轴。
	inline void WorldToScreen(Eigen::Vector4f& v);

private:
	Light light;
};
