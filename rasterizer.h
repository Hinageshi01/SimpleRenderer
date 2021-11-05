#pragma once

#include "global.h"
#include "model.h"

class Rasterizer
{
public:
	Rasterizer(const Light& l);
	
	// Bresenham �㷨��ֱ�ߡ�
	void BHLine(const Eigen::Vector4f& point0, const Eigen::Vector4f& point1, COLORREF color);
	// ������������ a��b��c ����ֵ���������޹ء� 
	inline Eigen::Vector3f BarycentricCoor(const float& x, const float& y, const Eigen::Vector4f* v);
	// ͨ���������������������һ��Ĳ�ֵ��
	inline Eigen::Vector4f Interpolate(const float& a, const float& b, const float& c, const Eigen::Vector4f& v1, const Eigen::Vector4f& v2, const Eigen::Vector4f& v3);
	// ͨ������ж�һ���Ƿ����������ڡ�
	inline bool InsideTriangle(const Eigen::Vector4f* v, const Eigen::Vector3f& p);
	// ��Χ�й�դ�������Ρ�
	void RasterizeTriangle_AABB(Eigen::Vector4f* v, Eigen::Vector4f* n, float* z_bufer);
	// ɨ���߹�դ�������Ρ�
	void RasterizeTriangle_SL(Eigen::Vector4f* v, Eigen::Vector4f* n, float* z_bufer);
	// ����ͶӰ����������Ļ��С�������� z �ᡣ
	inline void WorldToScreen(Eigen::Vector4f& v);

private:
	Light light;
};
