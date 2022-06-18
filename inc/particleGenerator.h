#pragma once

#include <iostream>
#include <vector>
#include <chrono>
#include <random>

#include <graphics.h>
#include <eigen3/Eigen/Eigen>

#include "inc/global.h"

struct Particle {
	float life;
	Eigen::Vector2f pos, speed;
	Eigen::Vector3f color;

	Particle();
};

class ParticleGenerator {
public:
	ParticleGenerator(const unsigned int amountIn, const float x, const float y);

	// ���� parCnt �������ӣ����»���������
	void UpDate(const float dt, const int parCnt);
	// ��������
	void Draw();

private:
	// ��������
	unsigned int amount;
	// ��¼��һ�������������������Ա������ҵ���һ����������������
	unsigned int lastUsedParticle = 0;
	// �������ӵ������εı߳�
	const int LENGTH = 4;
	// ��������λ��
	Eigen::Vector2f pos;
	// ���е�����
	std::vector<Particle> particles;

	// ���������
	std::default_random_engine generator;
	std::uniform_real_distribution<float> rPos;
	std::uniform_real_distribution<float> rColor;

	// ��ʼ�� amount ������
	void Init();
	// �����������������ӵ�����
	int FirstUnusedParticle();
	// Ϊ�µ����Ӹ�ֵ
	void RespawnParticle(Particle &par);

	void DrawSquare(const int index);
};
