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

	// 产生 parCnt 个新粒子，更新还存活的粒子
	void UpDate(const float dt, const int parCnt);
	// 绘制粒子
	void Draw();

private:
	// 粒子总数
	unsigned int amount;
	// 记录上一个消亡的粒子索引，以便更快的找到下一个消亡的粒子索引
	unsigned int lastUsedParticle = 0;
	// 代表粒子的正方形的边长
	const int LENGTH = 4;
	// 生成器的位置
	Eigen::Vector2f pos;
	// 所有的粒子
	std::vector<Particle> particles;

	// 随机数引擎
	std::default_random_engine generator;
	std::uniform_real_distribution<float> rPos;
	std::uniform_real_distribution<float> rColor;

	// 初始化 amount 个粒子
	void Init();
	// 返回最早消亡的粒子的索引
	int FirstUnusedParticle();
	// 为新的粒子赋值
	void RespawnParticle(Particle &par);

	void DrawSquare(const int index);
};
