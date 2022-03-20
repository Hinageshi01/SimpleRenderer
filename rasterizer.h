#pragma once

#include <graphics.h>

#include "model.h"
#include "vertexShader.h"

class Rasterizer
{
public:
	Rasterizer(const Light &l, const Eigen::Vector3f &e);
	
	// Bresenham �㷨��ֱ�ߡ�
	void BHLine(const Eigen::Vector4f &point0, const Eigen::Vector4f &point1);
	// ������������ a��b��c ����ֵ���������޹ء� 
	inline Eigen::Vector3f BarycentricCoor(const float &x, const float &y, const Vertex &v0, const Vertex &v1, const Vertex &v2);
	// ͨ���������������������һ��Ĳ�ֵ��
	template <typename T>
		inline T Interpolate(const float &a, const float &b, const float &c, const T &v1, const T &v3, const T &v2);
	// ɨ���߹�դ�������Ρ�
	void RasterizeTriangle_SL(Vertex *vertex, Model *model, float *z_bufer);
	// ����ͶӰ����������Ļ��С�������� z �ᡣ
	void WorldToScreen(Eigen::Vector4f &v);

private:
	Light light;
	Eigen::Vector3f eyePos;
};
