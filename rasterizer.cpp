#include "rasterizer.h"

#undef max
#undef min

Rasterizer::Rasterizer() {

}

void Rasterizer::SetLight(const Light& l) {
    light = l;
}

void Rasterizer::BHLine(const Eigen::Vector4f& point0, const Eigen::Vector4f& point1, COLORREF color) {
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
    for (int x = p0[0]; x <= p1[0]; x++) {
        if (steep) {
            putpixel(y, x, color);
        }
        else {
            putpixel(x, y, color);
        }
        dk += dy2;
        if (dk > 0) {
            y += (p1[1] > p0[1] ? 1 : -1);
            dk -= dx2;
        }
    }
}

inline Eigen::Vector3f Rasterizer::BarycentricCoor(const float& x, const float& y, const Eigen::Vector4f* v) {
    float c1 = (x * (v[1][1] - v[2][1]) + (v[2][0] - v[1][0]) * y + v[1][0] * v[2][1] - v[2][0] * v[1][1]) / (v[0][0] * (v[1][1] - v[2][1]) + (v[2][0] - v[1][0]) * v[0][1] + v[1][0] * v[2][1] - v[2][0] * v[1][1]);
    float c2 = (x * (v[2][1] - v[0][1]) + (v[0][0] - v[2][0]) * y + v[2][0] * v[0][1] - v[0][0] * v[2][1]) / (v[1][0] * (v[2][1] - v[0][1]) + (v[0][0] - v[2][0]) * v[1][1] + v[2][0] * v[0][1] - v[0][0] * v[2][1]);
    float c3 = (x * (v[0][1] - v[1][1]) + (v[1][0] - v[0][0]) * y + v[0][0] * v[1][1] - v[1][0] * v[0][1]) / (v[2][0] * (v[0][1] - v[1][1]) + (v[1][0] - v[0][0]) * v[2][1] + v[0][0] * v[1][1] - v[1][0] * v[0][1]);
    return Eigen::Vector3f(c1, c2, c3);
}

inline Eigen::Vector4f Rasterizer::Interpolate(const float& a, const float& b, const float& c, const Eigen::Vector4f& v1, const Eigen::Vector4f& v2, const Eigen::Vector4f& v3) {
    return a * v1 + b * v2 + c * v3;
}

inline bool Rasterizer::InsideTriangle(const Eigen::Vector4f* v, const Eigen::Vector3f p) {
    //TODO：优化一下矩阵运算
    Eigen::Vector3f a(v[0][0], v[0][1], 0);
    Eigen::Vector3f b(v[1][0], v[1][1], 0);
    Eigen::Vector3f c(v[2][0], v[2][1], 0);

    Eigen::Vector3f ab = b - a;
    Eigen::Vector3f bc = c - b;
    Eigen::Vector3f ca = a - c;

    Eigen::Vector3f ap = p - a;
    Eigen::Vector3f bp = p - b;
    Eigen::Vector3f cp = p - c;

    Eigen::Vector3f crs1 = ab.cross(ap);
    Eigen::Vector3f crs2 = bc.cross(bp);
    Eigen::Vector3f crs3 = ca.cross(cp);

    return ((crs1[2] >= 0 && crs2[2] >= 0 && crs3[2] >= 0) ||
            (crs1[2] <= 0 && crs2[2] <= 0 && crs3[2] <= 0));
}

void Rasterizer::RasterizeTriangle_AABB(Eigen::Vector4f* v, Eigen::Vector4f* n, float* z_bufer) {
    Eigen::Vector3f a(v[0][0], v[0][1], 0.f);
    Eigen::Vector3f b(v[1][0], v[1][1], 0.f);
    Eigen::Vector3f c(v[2][0], v[2][1], 0.f);

    Eigen::Vector3f ab = b - a;
    Eigen::Vector3f bc = c - b;
    Eigen::Vector3f faceNormal = ab.cross(bc);
    if (faceNormal[2] < 0) {
        //TODO：这里只关心z分量，优化一下矩阵运算
        // 当三角面背对视点时，不做光栅化。
        return;
    }

    if ((v[0][1] == v[1][1] && v[0][1] == v[2][1]) || (v[0][0] == v[1][0] && v[0][0] == v[2][0])) {
        // 三角面完全侧对视点时，不做光栅化。
        return;
    }

    int min_x = (int)std::floor(std::min(v[0][0], std::min(v[1][0], v[2][0])));
    int max_x = (int)std::ceil(std::max(v[0][0], std::max(v[1][0], v[2][0])));
    int min_y = (int)std::floor(std::min(v[0][1], std::min(v[1][1], v[2][1])));
    int max_y = (int)std::ceil(std::max(v[0][1], std::max(v[1][1], v[2][1])));

    min_x = std::clamp(min_x, 0, WIDTH - 1);
    max_x = std::clamp(max_x, 0, WIDTH - 1);
    min_y = std::clamp(min_y, 0, HEIGHT - 1);
    max_y = std::clamp(max_y, 0, HEIGHT - 1);

    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            if (InsideTriangle(v, Eigen::Vector3f(x, y, 0))) {
                int index = x + y * WIDTH;

                Eigen::Vector3f tmpBC = BarycentricCoor(x + 0.5f, y + 0.5f, v);
                float a = tmpBC[0];
                float b = tmpBC[1];
                float c = tmpBC[2];

                float a_div_w = a / (float)v[0][3];
                float b_div_w = b / (float)v[1][3];
                float c_div_w = c / (float)v[2][3];
                float z = 1.0f / (a_div_w + b_div_w + c_div_w);
                float zp = a_div_w * v[0][2] + b_div_w * v[1][2] + c_div_w * v[2][2];
                zp *= z;

                if (zp <= z_bufer[index]) {
                    z_bufer[index] = zp;

                    Eigen::Vector4f normal = Interpolate(a, b, c, n[0], n[1], n[2]);
                    float cosTerm = std::max(0.f, normal.head<3>().normalized().dot(light.direction.normalized()));
                    cosTerm += 0.1f;
                    cosTerm = std::clamp(cosTerm, 0.0f, 1.0f);
                    COLORREF color = RGB(cosTerm * light.intensity[0], cosTerm * light.intensity[1], cosTerm * light.intensity[2]);

                    putpixel(x, y, color);
                }
            }
        }
    }
}

void Rasterizer::RasterizeTriangle_SL(Eigen::Vector4f* v, Eigen::Vector4f* n, float* z_bufer) {
    Eigen::Vector3f a(v[0][0], v[0][1], 0.f);
    Eigen::Vector3f b(v[1][0], v[1][1], 0.f);
    Eigen::Vector3f c(v[2][0], v[2][1], 0.f);

    Eigen::Vector3f ab = b - a;
    Eigen::Vector3f bc = c - b;
    Eigen::Vector3f faceNormal = ab.cross(bc);
    if (faceNormal[2] < 0) {
        //TODO：这里只关心z分量，优化一下矩阵运算
        // 当三角面背对视点时，不做光栅化。
        return;
    }

    if ((v[0][1] == v[1][1] && v[0][1] == v[2][1]) || (v[0][0] == v[1][0] && v[0][0] == v[2][0])) {
        // 三角面完全侧对视点时，不做光栅化。
        return;
    }

    // 为三个点排序；v[0]，v[1]，v[2] 从下至上。
    if (v[0][1] > v[1][1]) {
        std::swap(v[0], v[1]);
        std::swap(n[0], n[1]);
    }
    if (v[0][1] > v[2][1]) {
        std::swap(v[0], v[2]);
        std::swap(n[0], n[2]);
    }
    if (v[1][1] > v[2][1]) {
        std::swap(v[1], v[2]);
        std::swap(n[1], n[2]);
    }

    int totalHeight = v[2][1] - v[0][1];
    int firstHeight = v[1][1] - v[0][1];
    int secondHeight = v[2][1] - v[1][1];
    for (int i = 0; i < totalHeight; i++) {
        bool isSecond = i > firstHeight || v[1][1] == v[0][1];
        int crtHeight = isSecond ? secondHeight : firstHeight;
        float alphaRate = i / (float)totalHeight;
        float betaRate = (i - (isSecond ? firstHeight : 0)) / (float)crtHeight;

        float A_x = v[0][0] + (v[2][0] - v[0][0]) * alphaRate;
        float B_x = isSecond ? v[1][0] + (v[2][0] - v[1][0]) * betaRate : v[0][0] + (v[1][0] - v[0][0]) * betaRate;

        // 这条扫描线从左（A）画向右（B）。
        if (A_x > B_x) std::swap(A_x, B_x);
        for (int j = A_x; j <= B_x; j++) {
            // 由外层的 i 确保光栅化每一行，而不是使用走样的  A.y 或 B.y。
            int x = j;
            int y = v[0][1] + i;
            int index = x + y * WIDTH;
            if (index > WIDTH * HEIGHT) {
                return;
            }

            Eigen::Vector3f tmpBC = BarycentricCoor(x + 0.5f, y + 0.5f, v);
            float a = tmpBC[0];
            float b = tmpBC[1];
            float c = tmpBC[2];

            float a_div_w = a / (float)v[0][3];
            float b_div_w = b / (float)v[1][3];
            float c_div_w = c / (float)v[2][3];
            float z = 1.0f / (a_div_w + b_div_w + c_div_w);
            float zp = a_div_w * v[0][2] + b_div_w * v[1][2] + c_div_w * v[2][2];
            zp *= z;

            if (zp <= z_bufer[index]) {
                z_bufer[index] = zp;

                Eigen::Vector4f tmpN = Interpolate(a, b, c, n[0], n[1], n[2]);
                Eigen::Vector3f normal(tmpN[0], tmpN[1], tmpN[2]);
                float cosTerm = std::max(0.f, normal.normalized().dot(light.direction.normalized()));
                cosTerm += 0.1f;
                cosTerm = std::clamp(cosTerm, 0.0f, 1.0f);
                COLORREF color = RGB(cosTerm * light.intensity[0], cosTerm * light.intensity[1], cosTerm * light.intensity[2]);

                putpixel(x, y, color);
            }
        }
    }
}

inline void Rasterizer::WorldToScreen(Eigen::Vector4f& v) {
    v = Eigen::Vector4f(int((v[0] + 1.) * WIDTH / 2. + 0.5), HEIGHT - int((v[1] + 1.) * HEIGHT / 2. + 0.5), v[2], 1.f);
}
