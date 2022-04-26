#include <iostream>
#include <chrono>

#include "inc/rasterizer.h"
#include "inc/vertexShader.h"

int main() {
    // 准备模型数据
    Model *model = nullptr;
    model = new Model("obj/african_head/african_head.obj");
    //model = new Model("obj/boggie/body.obj");
    //model = new Model("obj/diablo3_pose/diablo3_pose.obj");

    // 准备顶点着色器
    float angleY = 15.f;
    float scale = 3.5f;
    Eigen::Vector3f move(0, 0.1f, 0);
    Eigen::Vector3f eyePos(0, 0, 10);
    Frustum fru = {45.f, WIDTH / (float)HEIGHT, 0.1f, 50.f};
    VertexShader vs(angleY, scale, move, fru, eyePos);

    // 准备光栅化器
    Light light = {{-1,1,1}, {10, 10, 10}, {255,255,255}};
    Rasterizer r(light, eyePos);

    // 准备深度缓冲
    float *z_buffer = (float *)_mm_malloc(WIDTH * HEIGHT * sizeof(float), 4096);

    initgraph(WIDTH, HEIGHT);

    getmessage(EM_KEY);

    // 主循环。
    int sumFrame = 0;
    auto startTime = std::chrono::high_resolution_clock::now();
    while (true) {
        // 处理键盘输入
        flushmessage();
        unsigned char code = getmessage(EM_KEY).vkcode;
        if (code == VK_ESCAPE || code == VK_RETURN) {
            // 循环出口
            closegraph();
            break;
        }
        switch (code) {
            case 0x41: // A
                angleY += 4.f;
                break;
            case 0x44: // D
                angleY -= 4.f;
                break;
            case 0x53: // S
                scale -= 0.2f;
                break;
            case 0x57: // W
                scale += 0.2f;
                break;
        }
        vs.Update(angleY, scale);

        // 重置深度缓冲
        for (int i = 0; i < WIDTH * HEIGHT; ++i) {
            _mm_stream_si32((int *)&z_buffer[i], *(int *)&Z_MAX);
        }
        
        cleardevice();
        BeginBatchDraw();

        const int NUM_FACES = model->nfaces();
        for (int i = 0; i < NUM_FACES; ++i) {
            // 这个循环里遍历所有三角面，i 与 j 代表了第 i 个三角形的第 j 个顶点

            // 装配三角形
            Vertex vertexes[3];
#pragma unroll 3
            for (int j = 0; j < 3; ++j) {
                Eigen::Vector3f tmpP = model->vert(i, j);
                vertexes[j].pos = Eigen::Vector4f(tmpP[0], tmpP[1], tmpP[2], 1.f);

                Eigen::Vector3f tmpN = model->normal(i, j);
                vertexes[j].normal = Eigen::Vector4f(tmpN[0], tmpN[1], tmpN[2], 0.f);

                vertexes[j].uv = model->uv(i, j);
            }

            // 顶点着色器
            vs.Transform(vertexes);

            // 光栅化器与片元着色器
            r.RasterizeTriangle_SL(vertexes, model, z_buffer);
        }
        ++sumFrame;
        FlushBatchDraw();
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration = endTime - startTime;
    float sumTime = duration.count();

    std::cout << "Rendered " << sumFrame << " frames with averge " << sumTime / sumFrame << " s" << std::endl;
    std::cout << "FPS: " << sumFrame / sumTime << std::endl;
    return 0;
}
