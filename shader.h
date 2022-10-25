#include "tgaimage.h"
#include "geometry.h"
#include "model.h"

extern mat<4, 4> ModelView;
extern mat<4, 4> Projection;
extern mat<4, 4> Viewport;
extern vec3 light_dir;
extern vec3 eye;

void lookat(const vec3 &eye, const vec3 &origin, const vec3 &up);
void projection(double f=0.);
void viewport(int x, int y, int w, int h);
mat<3, 3> computeTangentBasis(std::vector<vec3> &vertices, std::vector<vec2> &uvs, vec3 &normal);

class IShader
{
public:
    virtual ~IShader(){};
    virtual vec4 vertex(int iface, int nthvert) = 0;
    virtual bool fragment(vec3 bar, TGAColor &color) = 0;
    static TGAColor sample2D(const TGAImage &img, vec2 &uvf) {
        return img.get(uvf[0] * img.width(), uvf[1] * img.height());
    }
};

vec3 barycentric(const vec4 &A, const vec4 &B, const vec4 &C, const vec2 &P);
void triangle(std::vector<vec4> &v, IShader &shader, TGAImage &image, TGAImage &zbuffer);

class GouraudShader : public IShader
{
private:
    Model *m;
    vec3 varying_intensity; // written by vertex shader, read by fragment shader
    mat<2, 3> varying_uv;

public:
    GouraudShader(Model *model) : m(model) {}
    ~GouraudShader(){}

    virtual vec4 vertex(int iface, int nthvert)
    {
        varying_intensity[nthvert] = std::max(0., m->normal(iface, nthvert) * light_dir);
        varying_uv.set_col(nthvert, m->uv(iface, nthvert));
        vec4 v = embed<4>(m->vert(iface, nthvert));
        return Viewport * Projection * ModelView * v;
    }

    virtual bool fragment(vec3 bar, TGAColor &color)
    {
        vec2 uv = varying_uv * bar;
        float intensity = varying_intensity * bar;
        /*
        if(intensity > .85) intensity = 1;
        else if(intensity > .60) intensity = .80;
        else if(intensity > .45) intensity = .60;
        else if(intensity > .30) intensity = .45;
        else if(intensity > .15) intensity = .20;
        else intensity = 0;
        */
        color = sample2D(m->diffuse(), uv);
        color = TGAColor(color[0] * intensity, color[1] * intensity, color[2] * intensity, color[3]);
        return false;
    }
};

class PhongShader : public IShader
{
private:
    Model *m;
    mat<3, 3> triangle;
    mat<3, 3> tbn;      // 3 basis of tangent space
    mat<2, 3> varying_uv;
    mat<3, 3> varying_nor;
    mat<4, 4> uniform_M;    // Projection * Modelview
    mat<4, 4> uniform_MIT;   // uniform_M.invert_transpose()

public:
    PhongShader(Model *model) : m(model) 
    {
        uniform_M = ModelView;
        uniform_MIT = uniform_M.invert_transpose();
    }
    ~PhongShader(){}

    virtual vec4 vertex(int iface, int nthvert)
    {
        vec4 v = embed<4>(m->vert(iface, nthvert));
        v = Viewport * Projection * ModelView * v;

        varying_uv.set_col(nthvert, m->uv(iface, nthvert));
        varying_nor.set_col(nthvert, m->normal(iface, nthvert));
        triangle.set_col(nthvert, proj<3>(v));

        return v;
    }

    virtual bool fragment(vec3 bar, TGAColor &color)
    {
        std::vector<vec3> vertices{triangle.col(0), triangle.col(1), triangle.col(2)};
        std::vector<vec2> uvs{varying_uv.col(0), varying_uv.col(1), varying_uv.col(2)};
        vec3 bn = varying_nor * bar;
        tbn = computeTangentBasis(vertices, uvs, bn);
        
        vec3 v = triangle * bar;
        vec2 uv = varying_uv * bar;
        vec3 n = (tbn * m->normal(uv)).normalize();
        vec3 l = proj<3>(uniform_M * embed<4>(light_dir, 0)).normalize();
        vec3 r = (2 * n * l * n - l).normalize();
        // vec3 camera_dir = (proj<3>(uniform_M * embed<4>(eye, 0)) - v).normalize();
        double ambient = 1.0f;
        double diffuse = std::max(0., n * l);
        double specular = pow(std::max(r.z, 0.), sample2D(m->specular(), uv)[0]);
        TGAColor ambient_color(10, 10, 10);
        color = sample2D(m->diffuse(), uv);
        // color = TGAColor{255, 255, 255};
        for(int i = 0; i < 3; ++i)
        {
            color[i] = std::min<int>(color[i] * (diffuse + .6 * specular) + ambient * ambient_color[i], 255);
        }
        return false;
    }
};