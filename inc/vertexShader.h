#pragma once

#include <eigen3/Eigen/Eigen>

#include "inc/global.h"

#undef near
#undef far

struct Vertex
{
	Eigen::Vector4f pos;
	Eigen::Vector4f normal;
	Eigen::Vector2f uv;
	// viewSpace ����ϵ�µĶ������ꡣ
	Eigen::Vector3f viewPos;
};

struct Frustum
{
	float fov;
	float aspect;
	float zNear;
	float zFar;
};

class VertexShader
{
public:
	VertexShader(const float _angleY, const float _scale, const Eigen::Vector3f &_move, const Frustum &_fru, const Eigen::Vector3f &_eyePos);

	// ����ѭ���б����ã����ż����������ģ�͵�λ����Ϣ��
	void Update(float a, float s);
	// �任����Ļ�ռ�����ϵ��
	void Transform(Vertex *v) const;

private:
	// ���� model ����
	Eigen::Matrix4f GetModelMatrix() const;

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
