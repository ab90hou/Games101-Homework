// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;

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

int CrossProduct(Eigen::Vector2f v1, Eigen::Vector2f v2)
{
    return (v1.x() * v2.y() - v2.x() * v1.y())>=0?1:-1;
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f AB(_v[1].x()-_v[0].x(),_v[1].y()-_v[0].y());
    Eigen::Vector2f BC(_v[2].x()-_v[1].x(),_v[2].y()-_v[1].y());
    Eigen::Vector2f CA(_v[0].x()-_v[2].x(),_v[0].y()-_v[2].y());
    Eigen::Vector2f AO(x - _v[0].x(), y - _v[0].y());
    Eigen::Vector2f BO(x - _v[1].x(), y - _v[1].y());
    Eigen::Vector2f CO(x - _v[2].x(), y - _v[2].y());

    int test1 = CrossProduct(AB, AO);
    int test2 = CrossProduct(CA, CO);
    int test3 = CrossProduct(BC, BO);
    if( test1*test2 >=0 && test1*test3 >= 0 && test2*test3 >=0 )
    {
        return true;
    }

    return false;

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
            vert.z() = -vert.z() * f1 + f2;
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
    array<vector<float>, 4> dir;
    dir[0] = { -0.25, -0.25 };
    dir[1] = { -0.25, 0.25 };
    dir[2] = { 0.25, -0.25 };
    dir[3] = { 0.25, 0.25 };

    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    Eigen::Vector3f a = t.a();
    Eigen::Vector3f b = t.b();
    Eigen::Vector3f c = t.c();

    float top,bottom,left,right;  //difine the bounding of box

    for(int ind = 0;ind<3;ind++)
    {
        left = std::min(left,t.v[ind][0]);
        right = std::max(right,t.v[ind][0]);
        top = std::max(top,t.v[ind][1]);
        bottom = std::min(bottom,t.v[ind][1]);
    }


    // iterate through the pixel and find if the current pixel is inside the triangle
    //scan from bottom point to top point along edge
    if( (left == a[0] && bottom == a[1]) || (left == b[0] && bottom == b[1]) || (left == c[0] && bottom == c[1]) )
    {   
        int scan_x = left;
        for(int ind_y = bottom ; ind_y<=top; ind_y++)
        {
            bool inside_flag = false; //false: outside true:inside
            for (int ind_x = scan_x; ind_x <= right; ind_x++)
            {
                //pixel is inside the triangle
                //开启ssaa
                if (SSAA_Flag)
                {
                    int count = 0;

                    float z_interpolated[4];
                    float z_interpolated_mix = 0.0f;
                    for (int ssaa_index = 0; ssaa_index < 4; ssaa_index++)
                    {
                        int new_x, new_y;
                        new_x = ind_x + dir[ssaa_index][0];
                        new_y = ind_y + dir[ssaa_index][1];
                       // = { std::numeric_limits<float>::infinity(),
                         //   std::numeric_limits<float>::infinity(),
                           // std::numeric_limits<float>::infinity(),
                            //std::numeric_limits<float>::infinity()};
                        if (insideTriangle(new_x, new_y, t.v))
                        {
                            count++;
                            //透视校正插值
                            //interpolate the color and z value, set its color on the screen
                            auto [alpha, beta, gamma] = computeBarycentric2D(new_x, new_y, t.v);
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            z_interpolated[ssaa_index] = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated[ssaa_index] *= w_reciprocal;
                            z_interpolated_mix += z_interpolated[ssaa_index];
                        }
                    }
                    if (count == 0)
                    {
                        if (inside_flag) //exit scan
                        {
                            break;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    if (!inside_flag)
                    {
                        inside_flag = !inside_flag;
                        scan_x = ind_x;  //record first inside point
                    }
                    
                    z_interpolated_mix /= count;

                    //比较并更新z-buffer
                    auto ind = get_index(ind_x, ind_y);

                    Eigen::Vector3f vertex(ind_x, ind_y, z_interpolated_mix);

                    if (depth_buf[ind] < z_interpolated_mix)
                    {
                        set_pixel(vertex, t.getColor()*count/4);
                        depth_buf[ind] = z_interpolated_mix;
                    }
                    continue;
                }
                else {

                    if (insideTriangle(ind_x, ind_y, t.v))
                    {
                        if (!inside_flag)
                        {
                            inside_flag = !inside_flag;
                            scan_x = ind_x;  //record first inside point
                        }

                        //透视校正插值
                        //interpolate the color and z value, set its color on the screen
                        auto [alpha, beta, gamma] = computeBarycentric2D(ind_x, ind_y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        //比较并更新z-buffer
                        auto ind = get_index(ind_x, ind_y);

                        Eigen::Vector3f vertex(ind_x, ind_y, z_interpolated);

                        if (depth_buf[ind] < z_interpolated)
                        {
                            set_pixel(vertex, t.getColor());
                            depth_buf[ind] = z_interpolated;
                        }

                        continue;
                    }
                }
                if(inside_flag) //exit scan
                {
                    break;
                }
                
            }
        }
    }
    //scan from top point to bottom point along edge
    else
    {
        int scan_x = left;
        for(int ind_y = top ; ind_y>=bottom; ind_y--)
        {
            bool inside_flag = false; //false: outside true:inside
            for(int ind_x = scan_x ; ind_x<=right ; ind_x++)
            {
                //pixel is inside the triangle
                //开启ssaa
                if (SSAA_Flag)
                {
                    int count = 0;

                    float z_interpolated[4];
                    float z_interpolated_mix;
                    for (int ssaa_index = 0; ssaa_index < 4; ssaa_index++)
                    {
                        int new_x, new_y;
                        new_x = ind_x + dir[ssaa_index][0];
                        new_y = ind_y + dir[ssaa_index][1];
                        // = { std::numeric_limits<float>::infinity(),
                          //   std::numeric_limits<float>::infinity(),
                            // std::numeric_limits<float>::infinity(),
                             //std::numeric_limits<float>::infinity()};
                        if (insideTriangle(new_x, new_y, t.v))
                        {
                            count++;
                            //透视校正插值
                            //interpolate the color and z value, set its color on the screen
                            auto [alpha, beta, gamma] = computeBarycentric2D(new_x, new_y, t.v);
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            z_interpolated[ssaa_index] = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated[ssaa_index] *= w_reciprocal;
                            z_interpolated_mix += z_interpolated[ssaa_index];
                        }
                    }
                    if (count == 0)
                    {
                        if (inside_flag) //exit scan
                        {
                            break;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    if (!inside_flag)
                    {
                        inside_flag = !inside_flag;
                        scan_x = ind_x;  //record first inside point
                    }

                    z_interpolated_mix /= count;

                    //比较并更新z-buffer
                    auto ind = get_index(ind_x, ind_y);

                    Eigen::Vector3f vertex(ind_x, ind_y, z_interpolated_mix);

                    if (depth_buf[ind] < z_interpolated_mix)
                    {
                        set_pixel(vertex, t.getColor() * count / 4);
                        depth_buf[ind] = z_interpolated_mix;
                    }
                    continue;
                }
                else {

                    if (insideTriangle(ind_x, ind_y, t.v))
                    {
                        if (!inside_flag)
                        {
                            inside_flag = !inside_flag;
                            scan_x = ind_x;  //record first inside point
                        }

                        //透视校正插值
                        //interpolate the color and z value, set its color on the screen
                        auto [alpha, beta, gamma] = computeBarycentric2D(ind_x, ind_y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        //比较并更新z-buffer
                        auto ind = get_index(ind_x, ind_y);

                        Eigen::Vector3f vertex(ind_x, ind_y, z_interpolated);

                        if (depth_buf[ind] < z_interpolated)
                        {
                            set_pixel(vertex, t.getColor());
                            depth_buf[ind] = z_interpolated;
                        }

                        continue;
                    }
                }
                if (inside_flag) //exit scan
                {
                    break;
                }
                
            }
        }
    }

    // If so, use the following code to get the interpolated z value.

    // auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    depth_buf.resize(w * h);
    frame_buf.resize(w* h);
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