#include <limits>
#include "shader.h"

mat<4, 4> ModelView;
mat<4, 4> Viewport;
mat<4, 4> Projection;

void lookat(const vec3 &eye, const vec3 &origin, const vec3 &up)
{
    vec3 z = (eye - origin).normalize();
    vec3 x = cross(up, z).normalize();
    vec3 y = cross(z, x).normalize();
    mat<4, 4> translation = {{{1, 0, 0, -origin.x}, {0, 1, 0, -origin.y}, {0, 0, 1, -origin.z}, {0, 0, 0, 1}}};
    mat<4, 4> rotation = {{{x.x, x.y, x.z, 0}, {y.x, y.y, y.z, 0}, {z.x, z.y, z.z, 0}, {0, 0, 0, 1}}};
    ModelView = rotation * translation;
}

void projection(double f)
{
    f = (f == 0) ? -1 : f;
    Projection = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, -1 / f, 1}}};
}

void viewport(int x, int y, int w, int h)
{
    int depth = 255;    // map z-buffer from [-1, 1] to [0, depth]
    Viewport = {{{w / 2., 0, 0, w / 2. + x}, {0, h / 2., 0, h / 2. + y}, {0, 0, depth / 2., depth / 2.}, {0, 0, 0, 1}}};
}

// compute bases of tangent space in triangle, which is tangent, bitangent and normal.
// return TBN matrix :
// [[tangent_x, bitangent_x, normal_x],
//  [tangent_y, bitangent_y, normal_y],
//  [tangent_z, bitangent_z, normal_z]]
mat<3, 3> computeTangentBasis(std::vector<vec3> &vertices, std::vector<vec2> &uvs, vec3 &normal)
{
    vec2 delta_uv1 = uvs[1] - uvs[0];
    vec2 delta_uv2 = uvs[2] - uvs[1];

    vec3 e1 = vertices[1] - vertices[0];
    vec3 e2 = vertices[2] - vertices[1];

    double r = 1. / (delta_uv1.x * delta_uv2.y - delta_uv2.x * delta_uv1.y);
    vec3 tangent = ((delta_uv2.y * e1 - delta_uv1.y * e2) * r).normalize();
    vec3 bitangent = ((-delta_uv2.x * e1 + delta_uv1.x * e2) * r).normalize();

    return mat<3, 3>{tangent, bitangent, normal}.transpose();
}

// There are 2 case to compute barycentric :
// 2D case: get propertive of vector AP in triangle ABC in basis AB and AC
// AP = u*AB + v*AC
// [u, v] = [AB, AC]^-1 * AP
// 
// 3D case: ray intersect with triangle
// P + v*t = a*A + b*B + r*C -> P + v*t = A + u*AB + v*AC -> AP = u*AB + v*AC + v*t
// [u, v, t] = [AB, AC, v]^-1 * AP
// 
// Here we use 2D case. (But we also need to do perspective revise)
// 3D case could be used when get intersection in ray tracing.
// 
// P = (1 - u -v)A + uB + vC
// return vector : [1 - u - v, u, v]
vec3 barycentric(const vec4 &A, const vec4 &B, const vec4 &C, const vec2 &P)
{
    // s = [AC.x, AB.x, PA.x
    //      AC.y, AB.y, PA.y]
    vec3 s[2];
    for(int i = 0; i < 2; ++i)
    {
        s[i][0] = C[i] / C[3] - A[i] / A[3];
        s[i][1] = B[i] / B[3] - A[i] / A[3];
        s[i][2] = A[i] / A[3] - P[i];
    }

    // uv[0] : S_ABP * 2
    // uv[1] : S_ACP * 2
    // uv[2] : S_ABC * 2
    vec3 uv = cross(s[0], s[1]);

    if(std::abs(uv[2]) > 1e-2)
    {
        double u = uv[1] / uv[2];
        double v = uv[0] / uv[2];
        double alpha = 1 - u - v, beta = u, gamma = v;

        // perspective revise(make 2d-barycentric to 3d-barycentric)
        double w_reciprocal = 1.0 / (alpha / A[3] + beta / B[3] + gamma / C[3]);
        alpha *= w_reciprocal / A[3];
        beta *= w_reciprocal / B[3];
        gamma *= w_reciprocal / C[3];
        return vec3(alpha, beta, gamma);
    }
    return vec3{-1, -1, -1};
}

void triangle(std::vector<vec4> &v, IShader &shader, TGAImage &image, TGAImage &zbuffer)
{
    int bbox_min[2] = {std::numeric_limits<int>::max()};
    int bbox_max[2] = {-std::numeric_limits<int>::max()};
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 2; ++j)
        {
            bbox_min[j] = std::min(bbox_min[j], (int)(v[i][j] / v[i][3]));
            bbox_max[j] = std::max(bbox_max[j], (int)(v[i][j] / v[i][3]));
        }
    }

    for(int x = bbox_min[0]; x <= bbox_max[0]; ++x)
    {
        for(int y = bbox_min[1]; y <= bbox_max[1]; ++y)
        {
            vec3 bc_screen = barycentric(v[0], v[1], v[2], vec2(x, y));
            if(bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0)
                continue;
            float z = bc_screen.x * v[0][2] + bc_screen.y * v[1][2] + bc_screen.z * v[2][2];
            float w = bc_screen.x * v[0][3] + bc_screen.y * v[1][3] + bc_screen.z * v[2][3];
            float depth = z / w;
            // std::cout << "z: " << z << "\nw: " << w << "\n";
            // std::cout << "bc_screen: " << bc_screen << "\n";
            // std::cout << "vertex[0]: " << v[0] << "\n";
            // std::cout << "vertex[1]: " << v[1] << "\n";
            // std::cout << "vertex[2]: " << v[2] << "\n";
            // std::cout << "depth: " << depth << " zbuffer: " << zbuffer.get(x, y)[0] << "\n";
            if(depth > zbuffer.get(x, y)[0])
            {
                TGAColor color;
                bool discard = shader.fragment(bc_screen, color);
                if(!discard)
                {
                    zbuffer.set(x, y, TGAColor(depth));
                    image.set(x, y, color);
                }
            }
        }
    }
}