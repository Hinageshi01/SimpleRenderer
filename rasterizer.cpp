#include "rasterizer.h"

#undef max
#undef min

Rasterizer::Rasterizer(const Light& l, const Eigen::Vector3f& e) :light(l), eyePos(e) {}

void Rasterizer::BHLine(const Eigen::Vector4f& point0, const Eigen::Vector4f& point1) {
    Eigen::Vector4f p0 = point0;
    Eigen::Vector4f p1 = point1;

    bool steep = false;
    if (std::abs(p0[1] - p1[1]) > std::abs(p0[0] - p1[0])) {
        // �� k > 1 ʱ������ת�û�� k < 1 �����������ʱ��Ҫ����Ϊ set(y, x)��
        std::swap(p0[0], p0[1]);
        std::swap(p1[0], p1[1]);
        steep = true;
    }
    if (p0[0] > p1[0]) {
        /* ����������Ϊ��������ʱ�������㽻����
           ����Ҫ��һ�����⴦����Ϊһ�������Ż����Ż���һ���ġ�*/
        std::swap(p0[0], p1[0]);
        std::swap(p0[1], p1[1]);
    }
    int dx = p1[0] - p0[0];
    int dy = p1[1] - p0[1];
    int dy2 = std::abs(dy) << 1;
    int dx2 = std::abs(dx) << 1;
    int dk = dy2 - dx;
    int y = p0[1];
    for (int x = p0[0]; x <= p1[0]; x++) {
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

inline Eigen::Vector3f Rasterizer::BarycentricCoor(const float& x, const float& y, const Vertex* v) {
    float c1 = (x * (v[1].pos[1] - v[2].pos[1]) + (v[2].pos[0] - v[1].pos[0]) * y + v[1].pos[0] * v[2].pos[1] - v[2].pos[0] * v[1].pos[1]) / (v[0].pos[0] * (v[1].pos[1] - v[2].pos[1]) + (v[2].pos[0] - v[1].pos[0]) * v[0].pos[1] + v[1].pos[0] * v[2].pos[1] - v[2].pos[0] * v[1].pos[1]);
    float c2 = (x * (v[2].pos[1] - v[0].pos[1]) + (v[0].pos[0] - v[2].pos[0]) * y + v[2].pos[0] * v[0].pos[1] - v[0].pos[0] * v[2].pos[1]) / (v[1].pos[0] * (v[2].pos[1] - v[0].pos[1]) + (v[0].pos[0] - v[2].pos[0]) * v[1].pos[1] + v[2].pos[0] * v[0].pos[1] - v[0].pos[0] * v[2].pos[1]);
    float c3 = (x * (v[0].pos[1] - v[1].pos[1]) + (v[1].pos[0] - v[0].pos[0]) * y + v[0].pos[0] * v[1].pos[1] - v[1].pos[0] * v[0].pos[1]) / (v[2].pos[0] * (v[0].pos[1] - v[1].pos[1]) + (v[1].pos[0] - v[0].pos[0]) * v[2].pos[1] + v[0].pos[0] * v[1].pos[1] - v[1].pos[0] * v[0].pos[1]);
    return Eigen::Vector3f(c1, c2, c3);
}

inline Eigen::Vector4f Rasterizer::Interpolate(const float& a, const float& b, const float& c, const Eigen::Vector4f& v1, const Eigen::Vector4f& v2, const Eigen::Vector4f& v3) {
    return a * v1 + b * v2 + c * v3;
}
inline Eigen::Vector3f Rasterizer::Interpolate(const float& a, const float& b, const float& c, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3) {
    return a * v1 + b * v2 + c * v3;
}
inline Eigen::Vector2f Rasterizer::Interpolate(const float& a, const float& b, const float& c, const Eigen::Vector2f& v1, const Eigen::Vector2f& v2, const Eigen::Vector2f& v3) {
    return a * v1 + b * v2 + c * v3;
}

void Rasterizer::RasterizeTriangle_SL(Vertex* v,  Model* const model, float* z_bufer) {
    float a_x = v[0].pos[0];
    float b_x = v[1].pos[0];
    float c_x = v[2].pos[0];
    float a_y = v[0].pos[1];
    float b_y = v[1].pos[1];
    float c_y = v[2].pos[1];

    // ���������� ab �� bc �����Ĳ�˲��жϽ���� z �����Ƿ�����Ļ��
    // ���ǲ��������淨������ xy �������ʼ�һ�����㡣
    float triangle_z = (b_x - a_x) * (c_y - b_y) - (b_y - a_y) * (c_x - b_x);
    if (triangle_z < 0) {
        // �������汳���ӵ�ʱ��������դ����
        return;
    }

    if ((v[0].pos[1] == v[1].pos[1] && v[0].pos[1] == v[2].pos[1]) || (v[0].pos[0] == v[1].pos[0] && v[0].pos[0] == v[2].pos[0])) {
        // ��������ȫ����ӵ�ʱ��������դ����
        return;
    }

    // Ϊ����������v[0]��v[1]��v[2] �������ϡ�
    if (v[0].pos[1] > v[1].pos[1]) {
        std::swap(v[0], v[1]);
    }
    if (v[0].pos[1] > v[2].pos[1]) {
        std::swap(v[0], v[2]);
    }
    if (v[1].pos[1] > v[2].pos[1]) {
        std::swap(v[1], v[2]);
    }

    // ������������ֱ��������һ�㣬�������׳��ֹ�դ������������
    v[0].pos[1] = std::floor(v[0].pos[1]);
    v[2].pos[1] = std::ceil(v[2].pos[1]);

    int totalHeight = v[2].pos[1] - v[0].pos[1] + 0.5f;
    int firstHeight = v[1].pos[1] - v[0].pos[1] + 0.5f;
    int secondHeight = v[2].pos[1] - v[1].pos[1] + 0.5f;
    for (int i = 1; i < totalHeight; i++) {
        bool isSecond = (i > firstHeight) || (v[1].pos[1] == v[0].pos[1]);
        int crtHeight = isSecond ? secondHeight : firstHeight;
        float alphaRate = i / (float)totalHeight;
        float betaRate = (i - (isSecond ? firstHeight : 0)) / (float)crtHeight;

        int A_x = v[0].pos[0] + (v[2].pos[0] - v[0].pos[0]) * alphaRate + 0.5f;
        int B_x = isSecond ? v[1].pos[0] + (v[2].pos[0] - v[1].pos[0]) * betaRate + 0.5f : v[0].pos[0] + (v[1].pos[0] - v[0].pos[0]) * betaRate + 0.5f;

        // ����ɨ���ߴ���A�������ң�B����
        if (A_x > B_x) std::swap(A_x, B_x);
        for (int j = A_x; j <= B_x; j++) {
            // ������ i ȷ����դ��ÿһ�У�������ʹ��������  A.y �� B.y��
            int x = j;
            int y = v[0].pos[1] + i;

            if (x < 0 || x > WIDTH - 1 || y < 0 || y > HEIGHT - 1) {
                continue;
            }

            // �����������ꡣ
            Eigen::Vector3f tmpBC = BarycentricCoor(x + 0.5f, y + 0.5f, v);
            float a = tmpBC[0];
            float b = tmpBC[1];
            float c = tmpBC[2];

            // �Ƚ���Ȳ�ֵ������ֱ����������Ȳ����޳���ƬԪ��
            float a_div_w = a / (float)v[0].pos[3];
            float b_div_w = b / (float)v[1].pos[3];
            float c_div_w = c / (float)v[2].pos[3];
            float z = 1.0f / (a_div_w + b_div_w + c_div_w);
            float zp = a_div_w * v[0].pos[2] + b_div_w * v[1].pos[2] + c_div_w * v[2].pos[2];
            zp *= z;

            int index = x + y * WIDTH;
            if (zp <= z_bufer[index]) {
                z_bufer[index] = zp;

                Eigen::Vector3f viewPos = Interpolate(a, b, c, v[0].viewPos, v[1].viewPos, v[2].viewPos);
                Eigen::Vector4f tmpN = Interpolate(a, b, c, v[0].normal, v[1].normal, v[2].normal);

                // ��Ҫ������������������ɫ��Ϊ�����㡣
                Eigen::Vector3f normal = tmpN.head<3>().normalized();
                Eigen::Vector3f viewDir = (eyePos - viewPos).normalized();
                Eigen::Vector3f lightDir = (light.position - viewPos).normalized();

                Eigen::Vector3f VL = viewDir + lightDir;
                Eigen::Vector3f halfDir = ((VL) / (VL).dot(VL)).normalized();

                // �� uv ��ֵ��
                Eigen::Vector2f uv = Interpolate(a, b, c, v[0].uv, v[1].uv, v[2].uv);

                // ���߿ռ䷨����ͼ��
                Eigen::Matrix3f A;
                Eigen::Vector3f tmp1 = v[1].viewPos - v[0].viewPos;
                Eigen::Vector3f tmp2 = v[2].viewPos - v[0].viewPos;
                A << tmp1[0], tmp1[1], tmp1[2],
                    tmp2[0], tmp2[1], tmp2[2],
                    normal[0], normal[1], normal[2];
                Eigen::Matrix3f AI = A.inverse();
                Eigen::Vector3f i = (AI * Eigen::Vector3f(v[1].uv[0] - v[0].uv[0], v[2].uv[0] - v[0].uv[0], 0)).normalized();
                Eigen::Vector3f j = (AI * Eigen::Vector3f(v[1].uv[1] - v[0].uv[1], v[2].uv[1] - v[0].uv[1], 0)).normalized();
                Eigen::Matrix3f B;
                B << i[0], i[1], i[2],
                    j[0], j[1], j[2],
                    normal[0], normal[1], normal[2];
                normal = (B * model->normalMap(uv)).normalized();

                // ȥ��ͼ�в�����Ȼ�� rgb ��Ϊ kd ʹ�á�
                unsigned char* bgra = model->diffuse(uv).bgra;
                int b = bgra[0];
                int g = bgra[1];
                int r = bgra[2];

                Eigen::Vector3f kd = Eigen::Vector3f(r, g, b) / 255.f;
                float ks = 0.3f;
                float ka = 0.02f;

                // ȥ�߹���ͼ�в�������Ϊ���� specular ��ʱ���ݴη�ʹ�á� 
                float p = model->specular(uv);
                float ld = std::max(0.f, normal.dot(lightDir));
                float ls = pow(std::max(0.f, normal.dot(halfDir)), p);

                Eigen::Vector3f inten;
                for (int k = 0; k < 3; k++) {
                    inten[k] = kd[k] * ld  + ks * ls + ka;
                    inten[k] = std::clamp(inten[k], 0.f, 1.f);
                }
                COLORREF color = RGB(inten[0] * 255, inten[1] * 255, inten[2] * 255);

                putpixel(x, y, color);
            }
        }
    }
}

inline void Rasterizer::WorldToScreen(Eigen::Vector4f& v) {
    v = Eigen::Vector4f(int((v[0] + 1.) * WIDTH / 2. + 0.5), HEIGHT - int((v[1] + 1.) * HEIGHT / 2. + 0.5), v[2], 1.f);
}

//inline bool Rasterizer::InsideTriangle(const Eigen::Vector4f* v, const Eigen::Vector3f& p) {
//    Eigen::Vector3f a(v[0][0], v[0][1], 0);
//    Eigen::Vector3f b(v[1][0], v[1][1], 0);
//    Eigen::Vector3f c(v[2][0], v[2][1], 0);
//
//    Eigen::Vector3f ab = b - a;
//    Eigen::Vector3f bc = c - b;
//    Eigen::Vector3f ca = a - c;
//
//    Eigen::Vector3f ap = p - a;
//    Eigen::Vector3f bp = p - b;
//    Eigen::Vector3f cp = p - c;
//
//    Eigen::Vector3f crs1 = ab.cross(ap);
//    Eigen::Vector3f crs2 = bc.cross(bp);
//    Eigen::Vector3f crs3 = ca.cross(cp);
//
//    return ((crs1[2] >= 0 && crs2[2] >= 0 && crs3[2] >= 0) ||
//            (crs1[2] <= 0 && crs2[2] <= 0 && crs3[2] <= 0));
//}
//
//void Rasterizer::RasterizeTriangle_AABB(Eigen::Vector4f* v, Eigen::Vector4f* n, float* z_bufer) {
//    Eigen::Vector3f a(v[0][0], v[0][1], 0.f);
//    Eigen::Vector3f b(v[1][0], v[1][1], 0.f);
//    Eigen::Vector3f c(v[2][0], v[2][1], 0.f);
//
//    Eigen::Vector3f ab = b - a;
//    Eigen::Vector3f bc = c - b;
//    Eigen::Vector3f faceNormal = ab.cross(bc);
//    if (faceNormal[2] < 0) {
//        //TODO������ֻ����z�������Ż�һ�¾�������
//        // �������汳���ӵ�ʱ��������դ����
//        return;
//    }
//
//    if ((v[0][1] == v[1][1] && v[0][1] == v[2][1]) || (v[0][0] == v[1][0] && v[0][0] == v[2][0])) {
//        // ��������ȫ����ӵ�ʱ��������դ����
//        return;
//    }
//
//    int min_x = (int)std::floor(std::min(v[0][0], std::min(v[1][0], v[2][0])));
//    int max_x = (int)std::ceil(std::max(v[0][0], std::max(v[1][0], v[2][0])));
//    int min_y = (int)std::floor(std::min(v[0][1], std::min(v[1][1], v[2][1])));
//    int max_y = (int)std::ceil(std::max(v[0][1], std::max(v[1][1], v[2][1])));
//
//    min_x = std::clamp(min_x, 0, WIDTH - 1);
//    max_x = std::clamp(max_x, 0, WIDTH - 1);
//    min_y = std::clamp(min_y, 0, HEIGHT - 1);
//    max_y = std::clamp(max_y, 0, HEIGHT - 1);
//
//    for (int x = min_x; x <= max_x; x++) {
//        for (int y = min_y; y <= max_y; y++) {
//            if (InsideTriangle(v, Eigen::Vector3f(x, y, 0))) {
//                int index = x + y * WIDTH;
//
//                Eigen::Vector3f tmpBC = BarycentricCoor(x + 0.5f, y + 0.5f, v);
//                float a = tmpBC[0];
//                float b = tmpBC[1];
//                float c = tmpBC[2];
//
//                float a_div_w = a / (float)v[0][3];
//                float b_div_w = b / (float)v[1][3];
//                float c_div_w = c / (float)v[2][3];
//                float z = 1.0f / (a_div_w + b_div_w + c_div_w);
//                float zp = a_div_w * v[0][2] + b_div_w * v[1][2] + c_div_w * v[2][2];
//                zp *= z;
//
//                if (zp <= z_bufer[index]) {
//                    z_bufer[index] = zp;
//
//                    Eigen::Vector4f normal = Interpolate(a, b, c, n[0], n[1], n[2]);
//                    float cosTerm = std::max(0.f, normal.head<3>().normalized().dot(light.direction.normalized()));
//                    cosTerm += 0.1f;
//                    cosTerm = std::clamp(cosTerm, 0.0f, 1.0f);
//                    COLORREF color = RGB(cosTerm * light.intensity[0], cosTerm * light.intensity[1], cosTerm * light.intensity[2]);
//
//                    putpixel(x, y, color);
//                }
//            }
//        }
//    }
//}
