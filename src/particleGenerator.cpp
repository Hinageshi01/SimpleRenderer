#include "inc/particleGenerator.h"

Particle::Particle() :
	life(0.f), pos(0.f, 0.f), speed(0.f, 0.f), color(0.f, 0.f, 0.f) { }

ParticleGenerator::ParticleGenerator(const unsigned int amountIn, const float x, const float y)
	: amount(amountIn), pos(x, y), rPos(-1.f, 1.f), rColor(0.5f, 1.f)
{
	Init();
}

void ParticleGenerator::Init() {
	particles.reserve(amount);
	for (int i = 0; i < amount; ++i) {
		particles.push_back(Particle());
	}
}

void ParticleGenerator::UpDate(const float dt, const int parCnt) {
	static const float MINUS_COLOR = 0.05f;
	static float sumTime = 0.f;

	// 更新生成器的位置，使其绕着屏幕中心旋转，半径硬编码在这里
	sumTime += dt;
	pos = Eigen::Vector2f(cos(sumTime), sin(sumTime)) * 128.f + Eigen::Vector2f(WIDTH / 2.f, HEIGHT / 2.f);

	// 生成 parCnt 个新粒子
	for (int i = 0; i < parCnt; ++i) {
		int usedParIndex = FirstUnusedParticle();
		RespawnParticle(particles[usedParIndex]);
	}

	// 更新还存活的粒子
	for (auto &par : particles) {
		par.life -= dt;
		if (par.life > 0.f) {
			par.pos += par.speed;
			// 速度衰减
			par.speed *= 0.9f;
			if (par.color[0] > MINUS_COLOR && par.color[1] > MINUS_COLOR && par.color[2] > MINUS_COLOR) {
				// 避免颜色产生负数
				par.color -= Eigen::Vector3f(MINUS_COLOR, MINUS_COLOR, MINUS_COLOR);
			}
		}
	}
}

int ParticleGenerator::FirstUnusedParticle() {
	// 下一个消亡的粒子的下标大概率是在上一个消亡粒子的右边
	for (int i = lastUsedParticle; i < amount; ++i) {
		if (particles[i].life <= 0.f) {
			lastUsedParticle = i;
			return i;
		}
	}
	// 如果找不到，就从头遍历
	for (int i = lastUsedParticle; i < lastUsedParticle; ++i) {
		if (particles[i].life <= 0.f) {
			lastUsedParticle = i;
			return i;
		}
	}
	// 由于生命太长或者生成太多，粒子不够用了
	lastUsedParticle = 0;
	return 0;
}

void ParticleGenerator::RespawnParticle(Particle &par) {
	const float RANGE = 10.f;

	// 重置粒子的生命
	par.life = 1.f;
	// 以生成器为基准，在一定范围内产生一个随机的位置
	Eigen::Vector2f rVec(rPos(generator), rPos(generator));
	if (rVec.norm() >= 1.f) {
		rVec.normalize();
	}
	par.pos = pos + rVec * RANGE;
	// 赋予一个离开生成器方向的速度，速度大小硬编码在这里
	par.speed = (par.pos - pos).normalized() * 10.f;
	// 产生一个随机的颜色
	par.color = Eigen::Vector3f(rColor(generator), rColor(generator), rColor(generator));
}

void ParticleGenerator::Draw() {
	// 从 lastUsedParticle 的位置开始绘制，保证新生成的粒子能覆盖旧粒子
	for (int i = lastUsedParticle; i < amount; ++i) {
		DrawSquare(i);
	}
	for (int i = 0; i < lastUsedParticle; ++i) {
		DrawSquare(i);
	}
}

void ParticleGenerator::DrawSquare(const int index) {
	Particle &par = particles[index];
	if (par.life > 0.f) {
		// 对于每个存活的粒子，绘制一个简单的正方形
		for (int y = par.pos[1] - LENGTH + 0.5f; y <= par.pos[1] + LENGTH + 0.5f; ++y) {
			for (int x = par.pos[0] - LENGTH + 0.5f; x <= par.pos[0] + LENGTH + 0.5f; ++x) {
				if (x<0 || x>WIDTH || y<0 || y>HEIGHT) {
					continue;
				}
				COLORREF color = RGB(par.color[0] * 255.f, par.color[1] * 255.f, par.color[2] * 255.f);
				putpixel(x, y, color);
			}
		}
	}
}
