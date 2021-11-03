#include <iostream>
#include <algorithm>
#include <graphics.h>
#include <conio.h>
#include "model.h"
#include <eigen3/Eigen/Eigen>
#include <omp.h>

#undef max
#undef min

const int WIDTH = 600;
const int HEIGHT = 600;
const double MY_PI = acos(-1);

struct Light
{
    Eigen::Vector3f direction;
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Model* model = nullptr;
Light light = Light{{ 1, 1, 1 }, { 1,2,3 },{ 255, 255, 255 }};

// Bresenham 算法画直线。
void BHLine(Eigen::Vector3f p0, Eigen::Vector3f p1, COLORREF color) {
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

// 通过叉乘判断一点是否在三角形内。
inline bool InsideTriangle(Eigen::Vector4f* v, Eigen::Vector3f P) {
    Eigen::Vector3f a(v[0][0], v[0][1], 0);
    Eigen::Vector3f b(v[1][0], v[1][1], 0);
    Eigen::Vector3f c(v[2][0], v[2][1], 0);

    Eigen::Vector3f ab = b - a;
    Eigen::Vector3f bc = c - b;
    Eigen::Vector3f ca = a - c;

    Eigen::Vector3f ap = P - a;
    Eigen::Vector3f bp = P - b;
    Eigen::Vector3f cp = P - c;

    Eigen::Vector3f crs1 = ab.cross(ap);
    Eigen::Vector3f crs2 = bc.cross(bp);
    Eigen::Vector3f crs3 = ca.cross(cp);

    return ((crs1[2] >= 0 && crs2[2] >= 0 && crs3[2] >= 0) ||
            (crs1[2] <= 0 && crs2[2] <= 0 && crs3[2] <= 0));
}

// 包围盒光栅化三角形。
void RasterizeTriangle_AABB(Eigen::Vector4f* v, float* z_bufer, COLORREF color) {
    int min_x = (int)std::floor(std::min(v[0][0], std::min(v[1][0], v[2][0])));
    int max_x = (int)std::ceil(std::max(v[0][0], std::max(v[1][0], v[2][0])));
    int min_y = (int)std::floor(std::min(v[0][1], std::min(v[1][1], v[2][1])));
    int max_y = (int)std::ceil(std::max(v[0][1], std::max(v[1][1], v[2][1])));

    min_x = min_x < 0 ? 0 : min_x;
    min_y = min_y < 0 ? 0 : min_y;
    max_x = max_x > WIDTH ? WIDTH : max_x;
    max_y = max_y > HEIGHT ? HEIGHT : max_y;

    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            if (InsideTriangle(v, Eigen::Vector3f(x, y, 0))) {
                putpixel(x, y, color);
            }
        }
    }
}

// 返回重心坐标 a、b、c 三个值，与向量无关。 
inline Eigen::Vector3f BarycentricCoor(float x, float y, const Eigen::Vector4f* v) {
    float c1 = (x * (v[1][1] - v[2][1]) + (v[2][0] - v[1][0]) * y + v[1][0] * v[2][1] - v[2][0] * v[1][1]) / (v[0][0] * (v[1][1] - v[2][1]) + (v[2][0] - v[1][0]) * v[0][1] + v[1][0] * v[2][1] - v[2][0] * v[1][1]);
    float c2 = (x * (v[2][1] - v[0][1]) + (v[0][0] - v[2][0]) * y + v[2][0] * v[0][1] - v[0][0] * v[2][1]) / (v[1][0] * (v[2][1] - v[0][1]) + (v[0][0] - v[2][0]) * v[1][1] + v[2][0] * v[0][1] - v[0][0] * v[2][1]);
    float c3 = (x * (v[0][1] - v[1][1]) + (v[1][0] - v[0][0]) * y + v[0][0] * v[1][1] - v[1][0] * v[0][1]) / (v[2][0] * (v[0][1] - v[1][1]) + (v[1][0] - v[0][0]) * v[2][1] + v[0][0] * v[1][1] - v[1][0] * v[0][1]);
    return Eigen::Vector3f(c1, c2, c3);
}

// 通过重心坐标计算三角形内一点的插值。
inline Eigen::Vector3f Interpolate(float a, float b, float c, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3) {
    return a * v1 + b * v2 + c * v3;
}

// 扫描线光栅化三角形。
void RasterizeTriangle(Eigen::Vector4f* v, Eigen::Vector3f* n, float* z_bufer) {
    if ((v[0][1] == v[1][1] && v[0][1] == v[2][1]) || (v[0][0] == v[1][0] && v[0][0] == v[2][0])) {
        // 三角形完全侧对视点时，不做光栅化。
        // TODO: 背面剔除
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

            Eigen::Vector3f tmp = BarycentricCoor(x, y, v);
            float a = tmp[0];
            float b = tmp[1];
            float c = tmp[2];

            float a_div_w = a / (float)v[0][3];
            float b_div_w = b / (float)v[1][3];
            float c_div_w = c / (float)v[2][3];
            float z = 1.0f / (a_div_w + b_div_w + c_div_w);
            float zp = a_div_w * v[0][2] + b_div_w * v[1][2] + c_div_w * v[2][2];
            zp *= z;

            if (zp > z_bufer[x + y * WIDTH]) {
                z_bufer[x + y * WIDTH] = zp;

                Eigen::Vector3f normal = Interpolate(a, b, c, n[0], n[1], n[2]);
                float cosTerm = std::max(0.f, normal.normalized().dot(light.direction.normalized()));
                cosTerm += 0.1f;
                cosTerm = std::clamp(cosTerm, 0.0f, 1.0f);
                COLORREF color = RGB(cosTerm * 255, cosTerm * 255, cosTerm * 255);

                putpixel(x, y, color);
                //std::cout << "hahaha" << std::endl;
            }
        }
    }
}

inline Eigen::Matrix4f GetModelMatrix(float angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.0f;
    rotation << cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;
    //Eigen::Matrix4f scale;
    //scale << 1, 0, 0, 0,
    //    0, 1, 0, 0,
    //    0, 0, 1, 0,
    //    0, 0, 0, 1 / 2.5;
    //Eigen::Matrix4f translate;
    //translate << 1, 0, 0, 0,
    //    0, 1, 0, 0,
    //    0, 0, 1, 0,
    //    0, 0, 0, 1;

    model = rotation * model;
    return model;
}

inline Eigen::Matrix4f GetViewMatrix(Eigen::Vector3f eyePos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eyePos[0],
        0, 1, 0, -eyePos[1],
        0, 0, 1, -eyePos[2],
        0, 0, 0, 1;

    view = translate * view;
    return view;
}

inline Eigen::Matrix4f GetProjectionMatrix(float fov, float aspect, float zNear, float zFar) {
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f Mpo;
    Mpo << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    fov = fov * MY_PI / 180.0f;
    float t = -1 * tan(fov / 2) * abs(zNear);
    float b = -t;
    float r = aspect * t;
    float l = -r;
    float n = zNear;
    float f = zFar;

    Eigen::Matrix4f Mo;
    Mo << 2 / (r - l), 0, 0, (r + l) / -2,
        0, 2 / (t - b), 0, (t + b) / -2,
        0, 0, 2 / (n - f), (n + f) / -2,
        0, 0, 0, 1;

    projection = Mo * Mpo * projection;
    return projection;
}

inline Eigen::Matrix4f GetViewportMatrix() {
    Eigen::Matrix4f viewport = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << WIDTH / 2.f, 0, 0, WIDTH / 2.f,
        0, HEIGHT / 2.f, 0, HEIGHT / 2.f,
        0, 0, 1, 0,
        0, 0, 1, 0;
}

inline Eigen::Vector4f world2screen(Eigen::Vector3f v) {
    return Eigen::Vector4f(int((v[0] + 1.) * WIDTH / 2. + 0.5), HEIGHT - int((v[1] + 1.) * HEIGHT / 2. + 0.5), v[2], 1.f);
}

int main() {

    model = new Model("obj/african_head/african_head.obj");
    //model = new Model("obj/boggie/body.obj");
    //model = new Model("obj/diablo3_pose/diablo3_pose.obj");
    initgraph(WIDTH, HEIGHT);
    
    float* z_buffer = new float[WIDTH * HEIGHT];
    for (int i = WIDTH * HEIGHT; i--; z_buffer[i] = -std::numeric_limits<float>::max());

    float startTime = omp_get_wtime();
#pragma omp parallel for
    for (int i = 0; i < model->nfaces(); i++) {
        // 这个循环里遍历所有三角面。
        std::vector<int> face = model->face(i);

        // TODO: 晚点把顶点数据封装成一个类
        Eigen::Vector4f vertex[3];
        for (int j = 0; j < 3; j++) {
            vertex[j] = world2screen(model->vert(face[j]));
        }
        Eigen::Vector3f normal[3];
        for (int j = 0; j < 3; j++) {
            normal[j] = model->normal(i, j).normalized();
        }

        RasterizeTriangle(vertex, normal, z_buffer);
    }
    float endTime = omp_get_wtime();
    std::cout << "Rendered 1 frame with " << endTime - startTime << " s" << std::endl;

    _getch();
    closegraph();

	return 0;
}
