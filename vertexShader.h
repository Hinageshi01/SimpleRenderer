#pragma once

#include "global.h"

#undef near
#undef far

class VertexShader
{
public:
	VertexShader(const float& a, const float& s, const Eigen::Vector3f& m, const Eigen::Vector3f& e, const Frustum& f);

	// ����λ����Ϣ
	void Update(const float& a, const Eigen::Vector3f& m);
	// model ����
	inline Eigen::Matrix4f GetModelMatrix();
	// view ����
	inline Eigen::Matrix4f GetViewMatrix();
	// �任����Ļ�ռ�����ϵ��
	void Transform(Eigen::Vector4f* vertex, Eigen::Vector4f* normal);

private:
	float angleY;
	float scale;

	Eigen::Vector3f move;
	Eigen::Vector3f eyePos;
	Frustum fru;

	float near;
	float far;

	// projection ���󲻳������ǲ���ı�ģ��ڹ��캯�������һ�μ��ɡ�
	Eigen::Matrix4f projection;
};
