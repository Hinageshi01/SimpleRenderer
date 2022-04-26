#include "inc/rasterizer.h"

#undef max
#undef min

Rasterizer::Rasterizer(const Light &_light, const Eigen::Vector3f &_eyePos) : light(_light), eyePos(_eyePos) { }

void Rasterizer::BHLine(const Eigen::Vector4f &point0, const Eigen::Vector4f &point1) const  {
    Eigen::Vector4f p0 = point0;
    Eigen::Vector4f p1 = point1;

    bool steep = false;
    if (std::abs(p0[1] - p1[1]) > std::abs(p0[0] - p1[0])) {
        // 当 k > 1 时，将其转置获得 k < 1 的线条，填充时需要调整为 set(y, x)
        std::swap(p0[0], p0[1]);
        std::swap(p1[0], p1[1]);
        steep = true;
    }
    if (p0[0] > p1[0]) {
        // 当线条方向为从右往左时，将两点交换，
        // 不需要进一步特殊处理，因为一条线正着画反着画是一样的。
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

    // 这里是在求 ab 与 bc 向量的叉乘并判断结果的 z 分量是否朝向屏幕，
    // 我们并不关心面法向量的 xy 分量，故简化一下运算。
    if ((v1.pos[0] - v0.pos[0]) * (v2.pos[1] - v1.pos[1]) - (v1.pos[1] - v0.pos[1]) * (v2.pos[0] - v1.pos[0]) < 0) {
        // 三角形背对视点时，不做光栅化
        return;
    }

    if ((v0.pos[0] == v1.pos[0] && v0.pos[1] == v1.pos[1]) ||
        (v1.pos[0] == v2.pos[0] && v1.pos[1] == v2.pos[1]) || 
        (v2.pos[0] == v0.pos[0] && v2.pos[1] == v0.pos[1])) {
        // 三角形没有面积时，不做光栅化
        return;
    }

    // 为三个点排序；v0，v1，v2 从下至上
    if (v0.pos[1] > v1.pos[1]) {
        std::swap(v0, v1);
    }
    if (v0.pos[1] > v2.pos[1]) {
        std::swap(v0, v2);
    }
    if (v1.pos[1] > v2.pos[1]) {
        std::swap(v1, v2);
    }

    // 将三角形向竖直方向拉伸一点，否则容易出现光栅化不到的像素
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

        // 这条扫描线从左（点A）画向右（点B）
        if (A_x > B_x) {
            std::swap(A_x, B_x);
        }

        for (int j = A_x; j <= B_x; ++j) {
            // 由外层的 i 确保光栅化每一行，而不是使用丢失精度的 A.y 或 B.y
            int x = j;
            int y = v0.pos[1] + i;

            if (x < 0 || x > WIDTH - 1 || y < 0 || y > HEIGHT - 1) {
                continue;
            }

            // 计算重心坐标以及透视矫正插值，先将深度插值出来，直接跳过被深度测试剔除的片元
            Eigen::Vector3f tmpBC = GetBarycentricCoor(x + 0.5f, y + 0.5f, v0, v1, v2);
            // pos[3] 存储的是 ViewSpace 中的 1/z
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

            // 重心坐标插值
            Eigen::Vector3f viewPos = Interpolate(baryCoor, v0.viewPos, v1.viewPos, v2.viewPos);
            Eigen::Vector4f tmpN = Interpolate(baryCoor, v0.normal, v1.normal, v2.normal);
            Eigen::Vector2f uv = Interpolate(baryCoor, v0.uv, v1.uv, v2.uv);

            // 重要的几个向量，均以着色点为出发点
            Eigen::Vector3f normal = tmpN.head(3).normalized();
            Eigen::Vector3f viewDir = (eyePos - viewPos).normalized();
            Eigen::Vector3f lightDir = (light.position - viewPos).normalized();
            Eigen::Vector3f halfDir = (viewDir + lightDir).normalized();

            // 切线空间法线贴图
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

            // 漫反射项
            // 去贴图中采样，然后将 rgb 作为 kd 使用
            auto *bgra = model->diffuse(uv).bgra;
            Eigen::Vector3f kd = Eigen::Vector3f(bgra[2], bgra[1], bgra[0]) / 255.f;
            float ld = std::max(0.f, normal.dot(lightDir));

            // 高光项
            // 去高光贴图中采样，作为计算 specular 项时的幂次方使用
            float ks = 0.3f;
            float ls = std::pow(std::max(0.f, normal.dot(halfDir)), model->specular(uv));

            // 环境光项
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
