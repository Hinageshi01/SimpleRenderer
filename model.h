#pragma once

#include <iostream>
#include <vector>

#include "global.h"
#include "tgaimage.h"

class Model
{
private:
    std::vector<Eigen::Vector3f> verts_;
    std::vector<std::vector<Eigen::Vector3i> > faces_; // attention, this Vec3i means vertex/uv/normal
    std::vector<Eigen::Vector3f> norms_;
    std::vector<Eigen::Vector2f> uv_;
    TGAImage diffusemap_;
    TGAImage normalmap_;
    TGAImage specularmap_;
    void load_texture(std::string filename, const char *suffix, TGAImage &img);
public:
    Model(const char *filename);
    ~Model();
    int nverts();
    int nfaces();
    Eigen::Vector3f normal(int iface, int nthvert);
    Eigen::Vector3f normalMap(Eigen::Vector2f uv);
    Eigen::Vector3f vert(int i);
    Eigen::Vector3f vert(int iface, int nthvert);
    Eigen::Vector2f uv(int iface, int nthvert);
    TGAColor diffuse(Eigen::Vector2f uv);
    float specular(Eigen::Vector2f uv);
    std::vector<int> face(int idx);
};
