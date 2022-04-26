#include "inc/rasterizer.h"

#undef max
#undef min

Rasterizer::Rasterizer(const Light &_light, const Eigen::Vector3f &_eyePos) : light(_light), eyePos(_eyePos) { }

void Rasterizer::BHLine(const Eigen::Vector4f &point0, const Eigen::Vector4f &point1) const  {
    Eigen::Vector4f p0 = point0;
    Eigen::Vector4f p1 = point1;

    bool steep = false;
    if (std::abs(p0[1] - p1[1]) > std::abs(p0[0] - p1[0])) {
        // �� k > 1 ʱ������ת�û�� k < 1 �����������ʱ��Ҫ����Ϊ set(y, x)
        std::swap(p0[0], p0[1]);
        std::swap(p1[0], p1[1]);
        steep = true;
    }
    if (p0[0] > p1[0]) {
        // ����������Ϊ��������ʱ�������㽻����
        // ����Ҫ��һ�����⴦����Ϊһ�������Ż����Ż���һ���ġ�
        std::swap(p0[0], p1[0]);
        std::swap(p0[1], p1[1]);
    }
    int dx = p1[0] - p0[0];
    int dy = p1[1] - p0[1];
    int dy2 = std::abs(dy) << 1;
    int dx2 = std::abs(dx) << 1;
    int dk = dy2 - dx;
    int y = p0[1];
    for (int x = p0[0]; x <= p1[0]; ++x) {
        if (steep) {
            putpixel(y, x, WHITE);
        }
        else {
            putpixel(x, y, WHITE);
        }
        dk += dy2;
        if (dk > 0) {
            y += (p1[1] > p0[1] ? 1 : -1);
            dk -= dx2;
        }
    }
}

Eigen::Vector3f Rasterizer::GetBarycentricCoor(const float &x, const float &y, const Vertex &v0, const Vertex &v1, const Vertex &v2) const {
    float squareDiv = 1.f /
        (v0.pos[0] * (v1.pos[1] - v2.pos[1]) + (v2.pos[0] - v1.pos[0]) * v0.pos[1] + v1.pos[0] * v2.pos[1] - v2.pos[0] * v1.pos[1]);
    float c1 = (x * (v1.pos[1] - v2.pos[1]) + (v2.pos[0] - v1.pos[0]) * y + v1.pos[0] * v2.pos[1] - v2.pos[0] * v1.pos[1]) * squareDiv;
    float c2 = (x * (v2.pos[1] - v0.pos[1]) + (v0.pos[0] - v2.pos[0]) * y + v2.pos[0] * v0.pos[1] - v0.pos[0] * v2.pos[1]) * squareDiv;
    return { c1, c2, (1.f - c1 - c2) };
}

void Rasterizer::RasterizeTriangle_SL(const Vertex const *vertex, Model *model, float *z_buffer) const  {
    Vertex v0 = vertex[0];
    Vertex v1 = vertex[1];
    Vertex v2 = vertex[2];

    // ���������� ab �� bc �����Ĳ�˲��жϽ���� z �����Ƿ�����Ļ��
    // ���ǲ��������淨������ xy �������ʼ�һ�����㡣
    if ((v1.pos[0] - v0.pos[0]) * (v2.pos[1] - v1.pos[1]) - (v1.pos[1] - v0.pos[1]) * (v2.pos[0] - v1.pos[0]) < 0) {
        // �����α����ӵ�ʱ��������դ��
        return;
    }

    if ((v0.pos[0] == v1.pos[0] && v0.pos[1] == v1.pos[1]) ||
        (v1.pos[0] == v2.pos[0] && v1.pos[1] == v2.pos[1]) || 
        (v2.pos[0] == v0.pos[0] && v2.pos[1] == v0.pos[1])) {
        // ������û�����ʱ��������դ��
        return;
    }

    // Ϊ����������v0��v1��v2 ��������
    if (v0.pos[1] > v1.pos[1]) {
        std::swap(v0, v1);
    }
    if (v0.pos[1] > v2.pos[1]) {
        std::swap(v0, v2);
    }
    if (v1.pos[1] > v2.pos[1]) {
        std::swap(v1, v2);
    }

    // ������������ֱ��������һ�㣬�������׳��ֹ�դ������������
    v0.pos[1] = std::floor(v0.pos[1]);
    v2.pos[1] = std::ceil(v2.pos[1]);

    float &left = const_cast<float &>(std::min(v0.pos[0], std::min(v1.pos[0], v2.pos[0])));
    float &right = const_cast<float &>(std::max(v0.pos[0], std::max(v1.pos[0], v2.pos[0])));
    
    //left = std::floor(left);
    //right = std::ceil(right);

    int totalHeight = v2.pos[1] - v0.pos[1] + 0.5f;
    int firstHeight = v1.pos[1] - v0.pos[1] + 0.5f;
    int secondHeight = v2.pos[1] - v1.pos[1] + 0.5f;
    for (int i = 1; i < totalHeight; ++i) {
        bool isSecond = (i > firstHeight) || (v1.pos[1] == v0.pos[1]);
        int crtHeight = isSecond ? secondHeight : firstHeight;
        float rate1 = i / (float)totalHeight;
        float rate2 = (i - (isSecond ? firstHeight : 0)) / (float)crtHeight;

        int A_x = v0.pos[0] + (v2.pos[0] - v0.pos[0]) * rate1 + 0.5f;
        int B_x = isSecond ? (v1.pos[0] + (v2.pos[0] - v1.pos[0]) * rate2 + 0.5f) : (v0.pos[0] + (v1.pos[0] - v0.pos[0]) * rate2 + 0.5f);

        // ����ɨ���ߴ��󣨵�A�������ң���B��
        if (A_x > B_x) {
            std::swap(A_x, B_x);
        }

        for (int j = A_x; j <= B_x; ++j) {
            // ������ i ȷ����դ��ÿһ�У�������ʹ�ö�ʧ���ȵ� A.y �� B.y
            int x = j;
            int y = v0.pos[1] + i;

            if (x < 0 || x > WIDTH - 1 || y < 0 || y > HEIGHT - 1) {
                continue;
            }

            // �������������Լ�͸�ӽ�����ֵ���Ƚ���Ȳ�ֵ������ֱ����������Ȳ����޳���ƬԪ
            Eigen::Vector3f tmpBC = GetBarycentricCoor(x + 0.5f, y + 0.5f, v0, v1, v2);
            // pos[3] �洢���� ViewSpace �е� 1/z
            Eigen::Vector3f baryCoor;
            baryCoor[0] = tmpBC[0] * v0.pos[3];
            baryCoor[1] = tmpBC[1] * v1.pos[3];
            baryCoor[2] = tmpBC[2] * v2.pos[3];
            baryCoor /= (baryCoor[0] + baryCoor[1] + baryCoor[2]);

            float z = Interpolate(baryCoor, v0.pos[2], v1.pos[2], v2.pos[2]);
            const int index = x + y * WIDTH;
            if (z > z_buffer[index]) {
                continue;
            }
            z_buffer[index] = z;

            // ���������ֵ
            Eigen::Vector3f viewPos = Interpolate(baryCoor, v0.viewPos, v1.viewPos, v2.viewPos);
            Eigen::Vector4f tmpN = Interpolate(baryCoor, v0.normal, v1.normal, v2.normal);
            Eigen::Vector2f uv = Interpolate(baryCoor, v0.uv, v1.uv, v2.uv);

            // ��Ҫ�ļ���������������ɫ��Ϊ������
            Eigen::Vector3f normal = tmpN.head(3).normalized();
            Eigen::Vector3f viewDir = (eyePos - viewPos).normalized();
            Eigen::Vector3f lightDir = (light.position - viewPos).normalized();
            Eigen::Vector3f halfDir = (viewDir + lightDir).normalized();

            // ���߿ռ䷨����ͼ
            Eigen::Vector2f dUV1 = v1.uv - v0.uv;
            Eigen::Vector2f dUV2 = v2.uv - v0.uv;
            float _u1 = dUV1[0];
            float _v1 = dUV1[1];
            float _u2 = dUV2[0];
            float _v2 = dUV2[1];
            float divTBN = 1.f / (_u1 * _v2 - _u2 * _v1);
            Eigen::Vector3f e1 = v1.viewPos - v0.viewPos;
            Eigen::Vector3f e2 = v2.viewPos - v0.viewPos;

            Eigen::Vector3f T(_v2 * e1[0] - _v1 * e2[0], _v2 * e1[1] - _v1 * e2[1], _v2 * e1[2] - _v1 * e2[2]);
            T *= divTBN;
            T.normalize();
            Eigen::Vector3f B(_u1 * e2[0] - _u2 * e1[0], _u1 * e2[1] - _u2 * e1[1], _u1 * e2[2] - _u2 * e1[2]);
            B *= divTBN;
            B.normalize();
            Eigen::Matrix3f TBN;
            TBN << T[0], B[0], normal[0],
                T[1], B[1], normal[1],
                T[2], B[2], normal[2];
            normal = (TBN * model->normalMap(uv)).normalized();

            // ��������
            // ȥ��ͼ�в�����Ȼ�� rgb ��Ϊ kd ʹ��
            auto *bgra = model->diffuse(uv).bgra;
            Eigen::Vector3f kd = Eigen::Vector3f(bgra[2], bgra[1], bgra[0]) / 255.f;
            float ld = std::max(0.f, normal.dot(lightDir));

            // �߹���
            // ȥ�߹���ͼ�в�������Ϊ���� specular ��ʱ���ݴη�ʹ��
            float ks = 0.3f;
            float ls = std::pow(std::max(0.f, normal.dot(halfDir)), model->specular(uv));

            // ��������
            float ka = 0.02f;

            float inten[3];
#pragma unroll 3
            for (int k = 0; k < 3; ++k) {
                inten[k] = std::clamp(kd[k] * ld + ks * ls + ka, 0.f, 1.f);
            }
            COLORREF color = RGB(inten[0] * 255.f, inten[1] * 255.f, inten[2] * 255.f);

            putpixel(x, y, color);
        }
    }
}

void Rasterizer::WorldToScreen(Eigen::Vector4f &v) const {
    v = Eigen::Vector4f(int((v[0] + 1.f) * WIDTH / 2.f + 0.5f), HEIGHT - int((v[1] + 1.f) * HEIGHT / 2.f + 0.5f), v[2], 1.f);
}
