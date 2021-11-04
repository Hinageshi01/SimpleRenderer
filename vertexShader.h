#pragma once

#include "global.h"

#undef near
#undef far

class VertexShader
{
public:
	VertexShader();
	VertexShader(const float& a, const float& s, const Eigen::Vector3f& m, const Eigen::Vector3f& e, const Frustum& f);

	// model ����
	inline Eigen::Matrix4f GetModelMatrix();
	// view ����
	inline Eigen::Matrix4f GetViewMatrix();
	// projection ����
	inline Eigen::Matrix4f GetProjectionMatrix();
	// �任����Ļ�ռ�����ϵ��
	void Transform(Eigen::Vector4f* vertex, Eigen::Vector4f* normal);

private:
	float angle;
	float scale;

	Eigen::Vector3f eyePos;
	Eigen::Vector3f move;
	Frustum fru;

	float radian;
	float top;
	float bottom;
	float right;
	float left;
	float near;
	float far;
};
