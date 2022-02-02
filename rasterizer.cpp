#include "rasterizer.h"

#undef max
#undef min

Rasterizer::Rasterizer(const Light &l, const Eigen::Vector3f &e) :light(l), eyePos(e) {}

void Rasterizer::BHLine(const Eigen::Vector4f &point0, const Eigen::Vector4f &point1) {
    Eigen::Vector4f p0 = point0;
    Eigen::Vector4f p1 = point1;

    bool steep = false;
    if (std::abs(p0[1] - p1[1]) > std::abs(p0[0] - p1[0])) {
        // 当 k > 1 时，将其转置获得 k < 1 的线条，填充时需要调整为 set(y, x)。
        std::swap(p0[0], p0[1]);
        std::swap(p1[0], p1[1]);
        steep = true;
    }
    if (p0[0] > p1[0]) {
        /* 当线条方向为从右往左时，将两点交换，
           不需要进一步特殊处理，因为一条线正着画反着画是一样的。*/
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

Eigen::Vector3f Rasterizer::BarycentricCoor(const float &x, const float &y, const Vertex* v) {
    float c1 = (x * (v[1].pos[1] - v[2].pos[1]) + (v[2].pos[0] - v[1].pos[0]) * y + v[1].pos[0] * v[2].pos[1] - v[2].pos[0] * v[1].pos[1]) / (v[0].pos[0] * (v[1].pos[1] - v[2].pos[1]) + (v[2].pos[0] - v[1].pos[0]) * v[0].pos[1] + v[1].pos[0] * v[2].pos[1] - v[2].pos[0] * v[1].pos[1]);
    float c2 = (x * (v[2].pos[1] - v[0].pos[1]) + (v[0].pos[0] - v[2].pos[0]) * y + v[2].pos[0] * v[0].pos[1] - v[0].pos[0] * v[2].pos[1]) / (v[1].pos[0] * (v[2].pos[1] - v[0].pos[1]) + (v[0].pos[0] - v[2].pos[0]) * v[1].pos[1] + v[2].pos[0] * v[0].pos[1] - v[0].pos[0] * v[2].pos[1]);
    float c3 = (x * (v[0].pos[1] - v[1].pos[1]) + (v[1].pos[0] - v[0].pos[0]) * y + v[0].pos[0] * v[1].pos[1] - v[1].pos[0] * v[0].pos[1]) / (v[2].pos[0] * (v[0].pos[1] - v[1].pos[1]) + (v[1].pos[0] - v[0].pos[0]) * v[2].pos[1] + v[0].pos[0] * v[1].pos[1] - v[1].pos[0] * v[0].pos[1]);
    return { c1, c2, c3 };
}

template <typename T>
T Rasterizer::Interpolate(const float &a, const float &b, const float &c, const T &v1, const T &v2, const T &v3) {
    return a * v1 + b * v2 + c * v3;
}

void Rasterizer::RasterizeTriangle_SL(Vertex *v, Model *model, float *z_bufer) {
    // 这里是在求 ab 与 bc 向量的叉乘并判断结果的 z 分量是否朝向屏幕。
    // 我们并不关心面法向量的 xy 分量，故简化一下运算。
    if ((v[1].pos[0] - v[0].pos[0]) * (v[2].pos[1] - v[1].pos[1]) - (v[1].pos[1] - v[0].pos[1]) * (v[2].pos[0] - v[1].pos[0]) < 0) {
        // 当三角面背对视点时，不做光栅化。
        return;
    }

    if ((v[0].pos[1] == v[1].pos[1] && v[0].pos[1] == v[2].pos[1]) || (v[0].pos[0] == v[1].pos[0] && v[0].pos[0] == v[2].pos[0])) {
        // 三角面完全侧对视点时，不做光栅化。
        return;
    }

    // 为三个点排序；v[0]，v[1]，v[2] 从下至上。
    if (v[0].pos[1] > v[1].pos[1]) {
        std::swap(v[0], v[1]);
    }
    if (v[0].pos[1] > v[2].pos[1]) {
        std::swap(v[0], v[2]);
    }
    if (v[1].pos[1] > v[2].pos[1]) {
        std::swap(v[1], v[2]);
    }

    // 将三角形向竖直方向拉伸一点，否则容易出现光栅化不到的像素
    v[0].pos[1] = std::floor(v[0].pos[1]);
    v[2].pos[1] = std::ceil(v[2].pos[1]);

    int totalHeight = v[2].pos[1] - v[0].pos[1] + 0.5f;
    int firstHeight = v[1].pos[1] - v[0].pos[1] + 0.5f;
    int secondHeight = v[2].pos[1] - v[1].pos[1] + 0.5f;
    for (int i = 1; i < totalHeight; ++i) {
        bool isSecond = (i > firstHeight) || (v[1].pos[1] == v[0].pos[1]);
        int crtHeight = isSecond ? secondHeight : firstHeight;
        float rate1 = i / (float)totalHeight;
        float rate2 = (i - (isSecond ? firstHeight : 0)) / (float)crtHeight;

        int A_x = v[0].pos[0] + (v[2].pos[0] - v[0].pos[0]) * rate1 + 0.5f;
        int B_x = isSecond ? (v[1].pos[0] + (v[2].pos[0] - v[1].pos[0]) * rate2 + 0.5f) : (v[0].pos[0] + (v[1].pos[0] - v[0].pos[0]) * rate2 + 0.5f);

        // 这条扫描线从左（点A）画向右（点B）。
        if (A_x > B_x) std::swap(A_x, B_x);
        for (int j = A_x; j <= B_x; ++j) {
            // 由外层的 i 确保光栅化每一行，而不是使用丢失精度的 A.y 或 B.y。
            int x = j;
            int y = v[0].pos[1] + i;

            if (x < 0 || x > WIDTH - 1 || y < 0 || y > HEIGHT - 1) {
                continue;
            }

            // 计算重心坐标以及透视矫正插值，先将深度插值出来，直接跳过被深度测试剔除的片元。
            Eigen::Vector3f tmpBC = BarycentricCoor(x + 0.5f, y + 0.5f, v);
            // pos[3] 存储的是 ViewSpace 中的 1/z。
            float a = tmpBC[0] * v[0].pos[3];
            float b = tmpBC[1] * v[1].pos[3];
            float c = tmpBC[2] * v[2].pos[3];
            float div = 1.f / (a + b + c);
            a *= div;
            b *= div;
            c *= div;
            float z = Interpolate(a, b, c, v[0].pos[2], v[1].pos[2], v[2].pos[2]);

            int index = x + y * WIDTH;
            if (z <= z_bufer[index]) {
                z_bufer[index] = z;
                
                // 重心坐标插值。
                Eigen::Vector3f viewPos = Interpolate(a, b, c, v[0].viewPos, v[1].viewPos, v[2].viewPos);
                Eigen::Vector4f tmpN = Interpolate(a, b, c, v[0].normal, v[1].normal, v[2].normal);
                Eigen::Vector2f uv = Interpolate(a, b, c, v[0].uv, v[1].uv, v[2].uv);

                // 重要的几个向量，均以着色点为出发点。
                Eigen::Vector3f normal = tmpN.head<3>().normalized();
                Eigen::Vector3f viewDir = (eyePos - viewPos).normalized();
                Eigen::Vector3f lightDir = (light.position - viewPos).normalized();
                Eigen::Vector3f halfDir = (viewDir + lightDir).normalized();

                // 切线空间法线贴图。
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

                // 漫反射项
                // 去贴图中采样，然后将 rgb 作为 kd 使用。
                unsigned char* bgra = model->diffuse(uv).bgra;
                Eigen::Vector3f kd = Eigen::Vector3f(bgra[2], bgra[1], bgra[0]) / 255.f;
                float ld = std::max(0.f, normal.dot(lightDir));

                // 高光项
                float ks = 0.3f;
                // 去高光贴图中采样，作为计算 specular 项时的幂次方使用。 
                float p = model->specular(uv);
                float ls = pow(std::max(0.f, normal.dot(halfDir)), p);

                // 环境光项
                float ka = 0.02f;

                Eigen::Vector3f inten;
                for (int k = 0; k < 3; ++k) {
                    inten[k] = std::clamp(kd[k] * ld + ks * ls + ka, 0.f, 1.f);
                }
                COLORREF color = RGB(inten[0] * 255, inten[1] * 255, inten[2] * 255);
                
                putpixel(x, y, color);
            }
        }
    }
}

void Rasterizer::WorldToScreen(Eigen::Vector4f &v) {
    v = Eigen::Vector4f(int((v[0] + 1.) * WIDTH / 2. + 0.5), HEIGHT - int((v[1] + 1.) * HEIGHT / 2. + 0.5), v[2], 1.f);
}
