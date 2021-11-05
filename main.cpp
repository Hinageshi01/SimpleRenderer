#include "rasterizer.h"
#include "vertexShader.h"

#undef max
#undef min

int main() {
    // 准备模型数据。
    Model* model = nullptr;
    model = new Model("obj/african_head/african_head.obj");
    //model = new Model("obj/boggie/body.obj");
    //model = new Model("obj/diablo3_pose/diablo3_pose.obj");

    // 准备顶点着色器。
    // TODO：物体会旋转，相机会移动，这个晚点要写到主循环里面
    float angleY = 20.f;
    float scale = 3.5f;
    Eigen::Vector3f move = {0,0.1f,0};
    Eigen::Vector3f eyePos = {0,0,10};
    Frustum fru = {45.f, 1.f, 0.1f, 50.f};
    VertexShader vs(angleY, scale, move, eyePos, fru);

    // 准备光栅化器。
    Light light = {{-1,1,1}, {5,5,1}, {150,200,250}};
    Rasterizer r(light, eyePos);
    
    // 准备深度缓冲。
    float* z_buffer = new float[WIDTH * HEIGHT];
    memset(z_buffer, std::numeric_limits<float>::max(), WIDTH * HEIGHT * sizeof(float));

    initgraph(WIDTH, HEIGHT);

    float startTime = omp_get_wtime();
#pragma omp parallel for
    for (int i = 0; i < model->nfaces(); i++) {
        // 这个循环里遍历所有三角面。
        std::vector<int> face = model->face(i);

        // 装配三角形。
        Vertex vertex[3];
        for (int j = 0; j < 3; j++) {
            Eigen::Vector3f tmpP = model->vert(face[j]);
            vertex[j].pos = Eigen::Vector4f(tmpP[0], tmpP[1], tmpP[2], 1.f);

            Eigen::Vector3f tmpN = model->normal(i, j);
            vertex[j].normal = Eigen::Vector4f(tmpN[0], tmpN[1], tmpN[2], 0.f);

            vertex[j].uv = model->uv(i, j);
        }

        // Vertex Shader
        vs.Transform(vertex);

        // 光栅化与 Fragment Shader
        r.RasterizeTriangle_SL(vertex, model, z_buffer);
    }
    float endTime = omp_get_wtime();
    std::cout << "Rendered 1 frame with " << endTime - startTime << " s" << std::endl;

    _getch();

    return 0;
}

// 调试起来别扭，最后再考虑把管线放入主循环
//int main() {
//    ExMessage m;
//
//    float sumTime = 0;
//    int sumFrame = 0;
//    BeginBatchDraw();
//    while (true) {
//        m = getmessage(EM_KEY);
//        if (m.message == WM_KEYDOWN) {
//            if (m.vkcode == VK_ESCAPE) {
//                closegraph();
//                break;
//            }
//            if (m.vkcode == 0x41) {
//                angle++;
//            }
//            if (m.vkcode == 0x44) {
//                angle--;
//            }
//        }
//        cleardevice();
//        memset(z_buffer, std::numeric_limits<float>::max(), WIDTH * HEIGHT * sizeof(float));
//        float startTime = omp_get_wtime();
//    #pragma omp parallel for
//        for (int i = 0; i < model->nfaces(); i++) {
//            // 这个循环里遍历所有三角面。
//        }
//        float endTime = omp_get_wtime();
//        sumTime += (endTime - startTime);
//        sumFrame++;
//        FlushBatchDraw();
//    }
//    EndBatchDraw();
//    std::cout << "Rendering 1 frame with averge " << sumTime / (float)sumFrame << " s" << std::endl;
//
//	return 0;
//}
