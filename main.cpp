#include <limits>
#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

int drawCount = 0;
int lineCount = 0;

// There are 2 case to compute barycentric :
// 2D case: get propertive of vector AP in triangle ABC in basis AB and AC
// AP = u*AB + v*AC
// [u, v] = [AB, AC]^-1 * AP
// 
// 3D case: ray intersect with triangle
// P + v*t = a*A + b*B + r*C -> P + v*t = A + u*AB + v*AC -> AP = u*AB + v*AC + v*t
// [u, v, t] = [AB, AC, v]^-1 * AP
// 
// Here we use 2D case.
// 3D case could be used when get intersection in ray tracing.
// 
// P = (1 - u -v)A + uB + vC
// return vector : [1 - u - v, u, v]
vec3 barycentric(const vec3 &A, const vec3 &B, const vec3 &C, const vec3 &P)
{
    // s = [AC.x, AB.x, PA.x
    //      AC.y, AB.y, PA.y]
    vec3 s[2];
    for(int i = 0; i < 2; ++i)
    {
        s[i][0] = C[i] - A[i];
        s[i][1] = B[i] - A[i];
        s[i][2] = A[i] - P[i];
    }

    // uv[0] : S_ABP * 2
    // uv[1] : S_ACP * 2
    // uv[2] : S_ABC * 2
    vec3 uv = cross(s[0], s[1]);

    if(std::abs(uv[2]) > 1e-2)
    {
        double u = uv[1] / uv[2];
        double v = uv[0] / uv[2];
        return vec3(1 - u - v, u, v);
    }
    return vec3{-1, -1, -1};
}

// Bresenham algorithm
void line(vec2 &p0, vec2 &p1, TGAImage &img, TGAColor color)
{
    bool steep = false; // dx > dy
    int x0 = p0.x, x1 = p1.x, y0 = p0.y, y1 = p1.y;
    if(std::abs(x1 - x0) < std::abs(y1 - y0))
    {
        steep = true;   // dy > dx
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if(x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dy = std::abs(y1 - y0), dx = std::abs(x1 - x0);
    int err = 0;
    int y = y0, y_step = y1 - y0 > 0 ? 1 : -1;
    
    for(int x = x0; x <= x1; ++x)
    {
        if(steep)
        {
            img.set(y, x, color);
            drawCount++;
        }
        else
        {
            img.set(x, y, color);
            drawCount++;
        }
        err += 2 * dy;
        if(err > dx)
        {
            y += y_step;
            err -= 2 * dx;
        }
    }
    lineCount++;
}

void triangle(vec3 &v0, vec3 &v1, vec3 &v2, std::vector<float> &z_buffer, TGAImage &img, TGAColor color)
{
    if(v0.y > v1.y) std::swap(v0, v1);
    if(v0.y > v2.y) std::swap(v0, v2);
    if(v1.y > v2.y) std::swap(v1, v2);

    int total_height = v2.y - v0.y;
    int segment_height = v1.y - v0.y;
    for(int i = 0; i <= total_height; ++i)
    {
        bool second_segment = (i >= segment_height) ? true : false;
        float alpha = (float)i / total_height;
        float beta = second_segment ? (float)(i - segment_height) / (total_height - segment_height) : (float)i / segment_height;
        vec3 A = v0 * (1 - alpha) + v2 * alpha;
        vec3 B = second_segment ? v1 * (1 - beta) + v2 * beta : v0 * (1 - beta) + v1 * beta;
        if(A.x > B.x) std::swap(A, B);
        for(int x = (int)A.x; x <= (int)B.x; ++x)
        {
            int y = v0.y + i;
            float z = 0; // todo
            if(z_buffer[x + y * width] < z)
            {
                z_buffer[x + y * width] = z;
                img.set(x, y, color);
            }
        }
    }
}

void Wireframe(Model *m, TGAImage &img)
{
    for(int i = 0; i < m->nfaces(); ++i)
    {
        for (int j=0; j<3; j++) { 
            vec3 v0 = m->vert(i, j); 
            vec3 v1 = m->vert(i, (j + 1) % 3); 
            vec2 p0((v0.x+1.)*width/2., (v0.y+1.)*height/2.);
            vec2 p1((v1.x+1.)*width/2., (v1.y+1.)*height/2.);
            line(p0, p1, img, white); 
        }
    } 
}

int main(int argc, char** argv)
{
    if(argc == 2)
    {
        model = new Model(argv[1]);
    }
    else
    {
        model = new Model("../obj/african_head/african_head.obj");
    }

    TGAImage framebuffer(width, height, TGAImage::RGB);
    /*
    std::vector<float> z_buffer(width * height, -std::numeric_limits<float>::max());
    vec3 light_dir(0, 0, -1);
    for(int i = 0; i < model->nfaces(); ++i)
    {
       // flat shading
       std::vector<vec3> v;
       for(int j = 0; j < 3; ++j)
       {
           vec3 v0 = model->vert(i, j);
           v0.x = (v0.x + 1) * width * 0.5;
           v0.y = (v0.y + 1) * height * 0.5;
           v.push_back(v0);
       }
       vec3 n;
       for(int j = 0; j < 3; ++j) n = n + model->normal(i, j) / 3;
       float intensity = std::max(n * light_dir, .0);
       TGAColor color(intensity * 255, intensity * 255, intensity * 255, 255);
       triangle(v[0], v[1], v[2], framebuffer, z_buffer, color);
    }  
    */
    /*
    vec2 p0(0.0, 5.0), p1(300.0, 200.0);
    line(p0, p1, framebuffer, white);
    p0 = {13, 20}; p1 = {80, 40};
    line(p0, p1, framebuffer, white); 
    p0 = {20, 13}; p1 = {40, 80};
    line(p0, p1, framebuffer, red); 
    p0 = {80, 40}; p1 = {13, 20};
    line(p0, p1, framebuffer, red);
    */
    Wireframe(model, framebuffer);
    std::cout << "The numbers of triangle is :" << model->nfaces() << std::endl;
    std::cout << "The drawCount is : " << drawCount << std::endl;
    std::cout << "The lineCount is : " << lineCount << std::endl;

    // frameBuffer.flip_vertically();
    framebuffer.write_tga_file("output.tga");
    delete model;
    return 0;
}