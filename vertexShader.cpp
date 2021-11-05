#include "vertexShader.h"

VertexShader::VertexShader(const float& a, const float& s, const Eigen::Vector3f& m, const Eigen::Vector3f& e, const Frustum& f) :
    angleY(a), scale(s), move(m), eyePos(e), fru(f)
{
    near = fru.zNear;
    far = fru.zFar;

    float radian = fru.fov * MY_PI / 180.f;
    float t = tan(radian / 2) * abs(fru.zNear);
    float b = -t;
    float r = fru.aspect * t;
    float l = -r;

    Eigen::Matrix4f Mpo;
    Mpo << fru.zNear, 0, 0, 0,
        0, fru.zNear, 0, 0,
        0, 0, fru.zNear + fru.zFar, -fru.zNear * fru.zFar,
        0, 0, 1, 0;
    Eigen::Matrix4f Mo;
    Mo << 2 / (r - l), 0, 0, (r + l) / -2,
        0, 2 / (t - b), 0, (t + b) / -2,
        0, 0, 2 / (near - far), (near + far) / -2,
        0, 0, 0, 1;

    projection = Mo * Mpo;
}

void VertexShader::Update(const float& a, const Eigen::Vector3f& m) {
    angleY = a;
    move = m;
}

inline Eigen::Matrix4f VertexShader::GetModelMatrix() {
    Eigen::Matrix4f Mrotation;
    float radian = angleY * MY_PI / 180.f;
    Mrotation << cos(radian), 0, sin(radian), 0,
        0, 1, 0, 0,
        -sin(radian), 0, cos(radian), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f Mscale;
    Mscale << 3.5, 0, 0, 0,
        0, 3.5, 0, 0,
        0, 0, 3.5, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f Mmove;
    Mmove << 1, 0, 0, move[0],
        0, 1, 0, move[1],
        0, 0, 1, move[2],
        0, 0, 0, 1;

    return Mmove * Mscale * Mrotation;
}

inline Eigen::Matrix4f VertexShader::GetViewMatrix() {
    Eigen::Matrix4f Mview = Eigen::Matrix4f::Identity();
    Mview << 1, 0, 0, -eyePos[0],
        0, 1, 0, -eyePos[1],
        0, 0, 1, -eyePos[2],
        0, 0, 0, 1;

    return Mview;
}

void VertexShader::Transform(Eigen::Vector4f* vertex, Eigen::Vector4f* normal) {
    Eigen::Matrix4f model = GetModelMatrix();
    Eigen::Matrix4f view = GetViewMatrix();

    Eigen::Matrix4f modelView = view * model;
    Eigen::Matrix4f mvp = projection * modelView;
    Eigen::Matrix4f modelViewInvTran = modelView.inverse().transpose();

    for (int i = 0; i < 3; i++) {
        Eigen::Vector4f v = mvp * vertex[i];

        v[0] /= v[3];
        v[1] /= v[3];
        v[2] /= v[3];
        v[3] /= v[3];

        const float f1 = (far - near) / 2.f;
        const float f2 = (far + near) / 2.f;
        v[0] = float(0.5f * WIDTH * (v[0] + 1.f));
        v[1] = float(0.5f * HEIGHT * (v[1] + 1.f));
        v[2] = v[2] * f1 + f2;
        vertex[i] = v;

        normal[i] = modelViewInvTran * normal[i];
    }
}
