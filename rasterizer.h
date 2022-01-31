#pragma once

#include "model.h"
#include "vertex.h"

class Rasterizer
{
public:
	Rasterizer(const Light &l, const Eigen::Vector3f &e);
	
	// Bresenham �㷨��ֱ�ߡ�
	void BHLine(const Eigen::Vector4f &point0, const Eigen::Vector4f &point1);
	// ������������ a��b��c ����ֵ���������޹ء� 
	Eigen::Vector3f BarycentricCoor(const float &x, const float &y, const Vertex* v);
	// ͨ���������������������һ��Ĳ�ֵ��
	Eigen::Vector4f Interpolate(const float &a, const float &b, const float &c, const Eigen::Vector4f &v1, const Eigen::Vector4f &v2, const Eigen::Vector4f &v3);
	Eigen::Vector3f Interpolate(const float &a, const float &b, const float &c, const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const Eigen::Vector3f &v3);
	Eigen::Vector2f Interpolate(const float &a, const float &b, const float &c, const Eigen::Vector2f &v1, const Eigen::Vector2f &v2, const Eigen::Vector2f &v3);
	float Interpolate(const float &a, const float &b, const float &c, const float& v1, const float& v2, const float& v3);
	// ɨ���߹�դ�������Ρ�
	void RasterizeTriangle_SL(Vertex *vertex, Model *model, float *z_bufer);
	// ����ͶӰ����������Ļ��С�������� z �ᡣ
	void WorldToScreen(Eigen::Vector4f &v);

private:
	Light light;
	Eigen::Vector3f eyePos;
};
