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

	// ������������λ�ã�ʹ��������Ļ������ת���뾶Ӳ����������
	sumTime += dt;
	pos = Eigen::Vector2f(cos(sumTime), sin(sumTime)) * 128.f + Eigen::Vector2f(WIDTH / 2.f, HEIGHT / 2.f);

	// ���� parCnt ��������
	for (int i = 0; i < parCnt; ++i) {
		int usedParIndex = FirstUnusedParticle();
		RespawnParticle(particles[usedParIndex]);
	}

	// ���»���������
	for (auto &par : particles) {
		par.life -= dt;
		if (par.life > 0.f) {
			par.pos += par.speed;
			// �ٶ�˥��
			par.speed *= 0.9f;
			if (par.color[0] > MINUS_COLOR && par.color[1] > MINUS_COLOR && par.color[2] > MINUS_COLOR) {
				// ������ɫ��������
				par.color -= Eigen::Vector3f(MINUS_COLOR, MINUS_COLOR, MINUS_COLOR);
			}
		}
	}
}

int ParticleGenerator::FirstUnusedParticle() {
	// ��һ�����������ӵ��±�����������һ���������ӵ��ұ�
	for (int i = lastUsedParticle; i < amount; ++i) {
		if (particles[i].life <= 0.f) {
			lastUsedParticle = i;
			return i;
		}
	}
	// ����Ҳ������ʹ�ͷ����
	for (int i = lastUsedParticle; i < lastUsedParticle; ++i) {
		if (particles[i].life <= 0.f) {
			lastUsedParticle = i;
			return i;
		}
	}
	// ��������̫����������̫�࣬���Ӳ�������
	lastUsedParticle = 0;
	return 0;
}

void ParticleGenerator::RespawnParticle(Particle &par) {
	const float RANGE = 10.f;

	// �������ӵ�����
	par.life = 1.f;
	// ��������Ϊ��׼����һ����Χ�ڲ���һ�������λ��
	Eigen::Vector2f rVec(rPos(generator), rPos(generator));
	if (rVec.norm() >= 1.f) {
		rVec.normalize();
	}
	par.pos = pos + rVec * RANGE;
	// ����һ���뿪������������ٶȣ��ٶȴ�СӲ����������
	par.speed = (par.pos - pos).normalized() * 10.f;
	// ����һ���������ɫ
	par.color = Eigen::Vector3f(rColor(generator), rColor(generator), rColor(generator));
}

void ParticleGenerator::Draw() {
	// �� lastUsedParticle ��λ�ÿ�ʼ���ƣ���֤�����ɵ������ܸ��Ǿ�����
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
		// ����ÿ���������ӣ�����һ���򵥵�������
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
