#pragma once

#include <graphics.h>

#include "model.h"
#include "vertexShader.h"

struct Light
{
	Eigen::Vector3f direction;
	Eigen::Vector3f position;
	Eigen::Vector3f intensity;
};

class Rasterizer
{
public:
	Rasterizer(const Light &_light, const Eigen::Vector3f &_eyePos);
	
	// Bresenham 算法画直线。
	void BHLine(const Eigen::Vector4f &point0, const Eigen::Vector4f &point1) const;
	// 扫描线光栅化三角形。
	void RasterizeTriangle_SL(const Vertex const *vertex, Model *model, float *z_buffer) const;
	// 正交投影并拉伸至屏幕大小，不处理 z 轴。
	void WorldToScreen(Eigen::Vector4f &v) const;

private:
	// 返回重心坐标 a、b、c 三个值，与向量无关。 
	inline Eigen::Vector3f GetBarycentricCoor(const float &x, const float &y, const Vertex &v0, const Vertex &v1, const Vertex &v2) const;
	// 通过重心坐标计算三角形内一点的插值。
	template <typename T>
	inline T Interpolate(const Eigen::Vector3f &bc, const T &v1, const T &v2, const T &v3) const {
		return bc[0] * v1 + bc[1] * v2 + bc[2] * v3;
	}

	Light light;
	Eigen::Vector3f eyePos;
};
