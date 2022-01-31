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
    float angleY = 15.f;
    float scale = 3.5f;
    Eigen::Vector3f move = {0,0.1f,0};
    Eigen::Vector3f eyePos = {0,0,10};
    Frustum fru = {45.f, WIDTH / (float)HEIGHT, 0.1f, 50.f};
    VertexShader vs(angleY, scale, move, eyePos, fru);

    // 准备光栅化器。
    Light light = {{-1,1,1}, {-10, 10, 10}, {150,200,250}};
    Rasterizer r(light, eyePos);
    
    // 准备深度缓冲。
    float* z_buffer = new float[WIDTH * HEIGHT];

    ExMessage msg;
    float sumTime = 0, sumFrame = 0;
    initgraph(WIDTH, HEIGHT);

    // 主循环。
    while (true) {
        // 处理键盘输入。
        flushmessage();
        msg = getmessage(EM_KEY);
        if (msg.vkcode == VK_ESCAPE || msg.vkcode == VK_RETURN) {
            closegraph();
            break;
        }
        switch (msg.vkcode) {
            case 0x41:
                angleY += 5.f;
                break;
            case 0x44:
                angleY -= 5.f;
                break;
            case 0x53:
                scale -= 0.3f;
                break;
            case 0x57:
                scale += 0.3f;
                break;
        }
        vs.Update(angleY, scale);

        memset(z_buffer, std::numeric_limits<float>::max(), WIDTH * HEIGHT * sizeof(float));
        
        cleardevice();
        BeginBatchDraw();

        float startTime = omp_get_wtime();
    #pragma omp parallel for
        for (int i = 0; i < model->nfaces(); ++i) {
            // 这个循环里遍历所有三角面。
            std::vector<int> face = model->face(i);

            // 装配三角形。
            Vertex vertex[3];
            for (int j = 0; j < 3; ++j) {
                Eigen::Vector3f tmpP = model->vert(face[j]);
                vertex[j].pos = Eigen::Vector4f(tmpP[0], tmpP[1], tmpP[2], 1.f);

                Eigen::Vector3f tmpN = model->normal(i, j);
                vertex[j].normal = Eigen::Vector4f(tmpN[0], tmpN[1], tmpN[2], 0.f);

                vertex[j].uv = model->uv(i, j);
            }

            // 顶点着色器。
            vs.Transform(vertex);

            // 光栅化器与片元着色器。
            r.RasterizeTriangle_SL(vertex, model, z_buffer);
        }
        float endTime = omp_get_wtime();
        sumTime += (endTime - startTime);
        ++sumFrame;
        FlushBatchDraw();
    }
    std::cout << "Rendered " << sumFrame << " frame with averge " << sumTime / sumFrame << " s" << std::endl;

    return 0;
}
