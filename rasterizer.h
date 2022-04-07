#pragma once

#include <graphics.h>

#include "model.h"
#include "vertexShader.h"

class Rasterizer
{
public:
	Rasterizer(const Light &l, const Eigen::Vector3f &e);
	
	// Bresenham �㷨��ֱ�ߡ�
	void BHLine(const Eigen::Vector4f &point0, const Eigen::Vector4f &point1) const;
	// ������������ a��b��c ����ֵ���������޹ء� 
	inline Eigen::Vector3f GetBarycentricCoor(const float &x, const float &y, const Vertex &v0, const Vertex &v1, const Vertex &v2) const;
	// ͨ���������������������һ��Ĳ�ֵ��
	template <typename T>
		inline T Interpolate(const Eigen::Vector3f bc, const T &v1, const T &v2, const T &v3) const;
	// ɨ���߹�դ�������Ρ�
	void RasterizeTriangle_SL(Vertex *vertex, Model *model, float *z_bufer) const;
	// ����ͶӰ����������Ļ��С�������� z �ᡣ
	void WorldToScreen(Eigen::Vector4f &v) const;

private:
	Light light;
	Eigen::Vector3f eyePos;
};
