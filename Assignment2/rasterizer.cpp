// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static float cross2d(const Vector2f& v1, const Vector2f& v2)
{
    return v1.x() * v2.y() - v1.y() * v2.x();
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f v0 = _v[0].head<2>(),
             v1 = _v[1].head<2>(), 
             v2 = _v[2].head<2>();
    Vector2f p(x, y);

    Vector2f v01 = v1 - v0, v12 = v2 - v1, v20 = v0 - v2,
             v0p = p - v0, v1p = p - v1, v2p = p - v2;

    float d1 = cross2d(v01, v0p), 
          d2 = cross2d(v12, v1p), 
          d3 = cross2d(v20, v2p);

    return ((d1 >= 0) && (d2 >= 0) && (d3 >= 0)) || ((d1 <= 0) && (d2 <= 0) && (d3 <= 0));
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    float x_min, x_max, y_min, y_max;
    x_min = floor(std::min(std::min(v[0].x(), v[1].x()), v[2].x()));
    x_max = ceil(std::max(std::max(v[0].x(), v[1].x()), v[2].x()));
    y_min = floor(std::min(std::min(v[0].y(), v[1].y()), v[2].y()));
    y_max = ceil(std::max(std::max(v[0].y(), v[1].y()), v[2].y()));

    // iterate through the pixel and find if the current pixel is inside the triangle
    // If so, use the following code to get the interpolated z value.
    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    Vector3f color = t.getColor();
    for (int y = y_min; y < y_max; y += 1)
    {
        for (int x = x_min; x < x_max; x += 1)
        {
            Vector2f p[4];
            p[0] = Vector2f(x + 0.25, y + 0.75);
            p[1] = Vector2f(x + 0.75, y + 0.75);
            p[2] = Vector2f(x + 0.25, y + 0.25);
            p[3] = Vector2f(x + 0.75, y + 0.25);

            float depth_min = std::numeric_limits<float>::infinity();
            bool inside = false;
            for (int i = 0; i < 4; i += 1)
            {
                if (insideTriangle(p[i].x(), p[i].y(), t.v))
                {
                    inside = true;

                    auto[alpha, beta, gamma] = computeBarycentric2D(p[i].x(), p[i].y(), t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if (-z_interpolated < depth_sample[get_index(x, y) * 4 + i])
                    {
                        depth_sample[get_index(x, y) * 4 + i] = -z_interpolated;
                        frame_sample[get_index(x, y) * 4 + i] = color / 4.0;
                    }
                    depth_min = std::min(depth_sample[get_index(x, y) * 4 + i], depth_min);
                }
            }

            if (inside) 
            {
                if (depth_min <= depth_buf[get_index(x, y)])
                {
                    Vector3f color_avg = frame_sample[get_index(x, y) * 4 + 0] + frame_sample[get_index(x, y) * 4 + 1] 
                                       + frame_sample[get_index(x, y) * 4 + 2] + frame_sample[get_index(x, y) * 4 + 3];    

                    depth_buf[get_index(x, y)] = depth_min;
                    set_pixel(Eigen::Vector3f(x, y, depth_min), color_avg);
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    frame_sample.resize(w * h * 4);
    depth_buf.resize(w * h);
    depth_sample.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on