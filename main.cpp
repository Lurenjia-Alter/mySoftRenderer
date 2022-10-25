#include <limits>
#include <vector>
#include <cmath>
#include "tgaimage.h"
// #include "model.h"
#include "geometry.h"
#include "shader.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

vec3 light_dir(0, 0, 1);
vec3 eye(0.5, 1, 2);
vec3 origin(0, 0, 1);
vec3 up(0, 1, 0);

/*
vec3 world2screen(vec3 v) {
    vec4 ret = Viewport * Projection * ModelView * vec4{v.x, v.y, v.z, 1.0};
    return vec3{ret[0] / ret[3], ret[1] / ret[3], ret[2] / ret[3]};
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
        }
        else
        {
            img.set(x, y, color);
        }
        err += 2 * dy;
        if(err > dx)
        {
            y += y_step;
            err -= 2 * dx;
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

void FlatShading(Model *m, TGAImage &img)
{
    std::vector<float> z_buffer(width * height, -std::numeric_limits<float>::max());
    for(int i = 0; i < m->nfaces(); ++i)
    {
       // flat shading
       std::vector<vec3> world_coords;
       std::vector<vec3> screen_coords;
       std::vector<vec2> texture_coords;
       for(int j = 0; j < 3; ++j)
       {
           vec3 v0 = m->vert(i, j);
           world_coords.push_back(v0);
           screen_coords.push_back(world2screen(v0));

           vec2 uv = m->uv(i, j);
           texture_coords.push_back(uv);
       }
       vec3 n = cross(world_coords[2] - world_coords[0], world_coords[1] - world_coords[0]).normalize();
       float intensity = std::max(0.0, n * light_dir);
       // TGAColor color(intensity, intensity, intensity, 255);
       triangle(m, screen_coords, texture_coords, z_buffer, img, intensity);
    }
}
*/

void shading(IShader &shader, TGAImage &framebuffer, TGAImage &zbuffer)
{
    for(int i = 0; i < model->nfaces(); ++i)
    {
        std::vector<vec4> screen_corrds;
        for(int j = 0; j < 3; ++j)
        {
            screen_corrds.push_back(shader.vertex(i, j));
        }
        triangle(screen_corrds, shader, framebuffer, zbuffer);
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

    std::cout << "Create frambuffer.\n";
    TGAImage framebuffer(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);
    std::cout << "Compute mvp matrix.\n";
    lookat(eye, origin, up);
    viewport(width/8, height/8, width*3/4, height*3/4);
    projection((eye - origin).norm());
    light_dir.normalize();

    std::cout << "Rendering.\n";
    // GouraudShader shader(model);
    PhongShader shader(model);
    shading(shader, framebuffer, zbuffer);
    std::cout << "Generate output image.\n";

    // frameBuffer.flip_vertically();
    zbuffer.write_tga_file("zbuffer.tga");
    framebuffer.write_tga_file("output.tga");
    std::cout << "Free memory.\n";
    delete model;
    return 0;
}