#pragma once

#include "vertex.h"

#undef near
#undef far

class VertexShader
{
public:
	VertexShader(const float &a, const float &s, const Eigen::Vector3f &m, const Eigen::Vector3f &e, const Frustum &f);

	// ����ѭ���б����ã����ż����������ģ�͵�λ����Ϣ��
	void Update(const float &a, const float &s);
	// ���� model ����
	Eigen::Matrix4f GetModelMatrix();
	// �任����Ļ�ռ�����ϵ��
	void Transform(Vertex *v);

private:
	// model ��������Ҫ�����ݡ�
	float angleY;
	float scale;
	Eigen::Vector3f move;

	// ƽ��ͷ�塣
	Frustum fru;

	// ���������󲻳������ǲ���ı�ģ�����͵����ֻ�ڹ��캯�������һ�Ρ�
	Eigen::Matrix4f view;
	Eigen::Matrix4f projection;
	Eigen::Matrix4f Mmove; // ����� model �����е��ƶ�����
};
