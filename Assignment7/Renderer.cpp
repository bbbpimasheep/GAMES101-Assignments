//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <omp.h>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.0001f, GAMMA_INV = 1.0f / 2.2f;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 1024, root = 32;
    std::cout << "SPP: " << spp << "\n";
    #pragma omp parallel for shared(m)
        for (uint32_t j = 0; j < scene.height; ++j) {
            #pragma omp parallel for
                for (uint32_t i = 0; i < scene.width; ++i) {
                    // generate primary ray direction
                    float unit = 1.0f / ((float)root + 1);
                    for (int k = 0; k < spp; k++){
                        int q = k / root + 1, r = k % root + 1;
                        float x = (2 * (i + unit * q) / (float)scene.width - 1) * imageAspectRatio * scale;
                        float y = (1 - 2 * (j + unit * r) / (float)scene.height) * scale;

                        Vector3f dir = normalize(Vector3f(-x, y, 1));
                        framebuffer[j * scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp; 
                    }
                }
            #pragma omp critical
            UpdateProgress(m / (float)scene.height);
            m += 1;
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), GAMMA_INV));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), GAMMA_INV));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), GAMMA_INV));
        // Gamma Correction
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
