/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TextureLT.cpp
 * Author: swl
 * 
 * Created on June 10, 2016, 11:44 AM
 */

#include <string.h>
#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION
#include "TextureLT.h"

RTCDevice TextureLT::device = NULL;
RTCScene TextureLT::scene = NULL;
float TextureLT::epsilon = 1e-2;
std::vector<tinyobj::shape_t> TextureLT::shapes;
std::vector<tinyobj::material_t> TextureLT::materials;
std::vector<std::vector<cv::Vec3f> > TextureLT::texel2coords;
unsigned TextureLT::width = 0;

void TextureLT::init()
{
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    device = rtcNewDevice(NULL);
}

void TextureLT::load_scene(const std::string& path)
{
    if(scene) rtcDeleteScene(scene);
    scene = rtcDeviceNewScene(device, RTC_SCENE_STATIC, RTC_INTERSECT1);
    shapes.clear();
    materials.clear();
    std::string::size_type pos = path.rfind('/');
    std::string mtl_base_path = "";
    if (pos != std::string::npos)
    {
        mtl_base_path = path.substr(0, pos + 1);
    }
    // load obj -- begin
    
    std::string err;
    bool ret = tinyobj::LoadObj(shapes, materials, err, path.c_str(), mtl_base_path.c_str());
    if (!err.empty())
    { // `err` may contain warning message.
        std::cerr << err << std::endl;
    }
    if (!ret)
    {
        exit(1);
    }
    // load obj -- end
    
    for(size_t shape_index = 0; shape_index < shapes.size(); shape_index++)
    {
        const tinyobj::mesh_t& mesh = shapes[shape_index].mesh;
        unsigned int geo_id = rtcNewTriangleMesh(scene, RTC_GEOMETRY_STATIC, 
                mesh.indices.size()/3, mesh.positions.size()/3);
        
        float* vertices = (float*) rtcMapBuffer(scene, geo_id, RTC_VERTEX_BUFFER); 
        for(int i=0; i<mesh.positions.size(); i++)
            vertices[(i/3)+i] = mesh.positions[i];
        rtcUnmapBuffer(scene, geo_id, RTC_VERTEX_BUFFER);

        unsigned* indices = (unsigned*) rtcMapBuffer(scene, geo_id, RTC_INDEX_BUFFER);
        memcpy(indices, mesh.indices.data(), mesh.indices.size()*sizeof(unsigned));
        rtcUnmapBuffer(scene, geo_id, RTC_INDEX_BUFFER);
    }
    rtcCommit(scene);
}

inline float det(const cv::Vec2f& col1, const cv::Vec2f& col2)
{
    return col1[0]*col2[1] - col1[1] * col2[0];
}

void TextureLT::fill_texel2coord(const std::string& shape_name, int w, int h)
{
    width = w;
    texel2coords.resize(w*h, std::vector<cv::Vec3f>());
    for(size_t shape_index = 0; shape_index < shapes.size(); shape_index++)
    {
        if(shapes[shape_index].name != shape_name) continue;
        const tinyobj::mesh_t& mesh = shapes[shape_index].mesh;
        for(size_t i = 0; i < mesh.indices.size()/3; i++)
        {
            unsigned i0 = mesh.indices[3*i];
            unsigned i1 = mesh.indices[3*i+1];
            unsigned i2 = mesh.indices[3*i+2];
            cv::Vec2f t0 = ((cv::Vec2f*)mesh.texcoords.data())[i0];
            cv::Vec2f t1 = ((cv::Vec2f*)mesh.texcoords.data())[i1];
            cv::Vec2f t2 = ((cv::Vec2f*)mesh.texcoords.data())[i2];
            cv::Vec3f v0 = ((cv::Vec3f*)mesh.positions.data())[i0];
            cv::Vec3f v1 = ((cv::Vec3f*)mesh.positions.data())[i1];
            cv::Vec3f v2 = ((cv::Vec3f*)mesh.positions.data())[i2];
            if(t0[0] > t1[0])
            {
                std::swap(t0, t1);
                std::swap(v0, v1);
            }
            if(t1[0] > t2[0])
            {
                std::swap(t1, t2);
                std::swap(v1, v2);
            }
            if(t0[0] > t1[0])
            {
                std::swap(t0, t1);
                std::swap(v0, v1);
            }
            
            t0[0] *= w; t0[1] *= h;
            t1[0] *= w; t1[1] *= h;
            t2[0] *= w; t2[1] *= h;
            
            cv::Vec2f t01 = t1 - t0;
            cv::Vec2f t02 = t2 - t0;
            for(int x = ceil(t0[0]); x<=floor(t2[0]); x++)
            {
                float ys = (t2[1]-t0[1]) / (t2[0]-t0[0]) * (x - t0[0]) + t0[1];
                float ye = x < t1[0] ?
                    (t1[1]-t0[1]) / (t1[0]-t0[0]) * (x - t0[0]) + t0[1] :
                    (t2[1]-t1[1]) / (t2[0]-t1[0]) * (x - t1[0]) + t1[1];
                if(ys > ye)
                    std::swap(ys, ye);
                for(int y = ceil(ys); y <= floor(ye); y++)
                {
                    if(x < 0 || x >= w)
                        continue;
                    if(y < 0 || y >= h)
                        continue;
                    cv::Vec2f p = cv::Vec2f(x, y) - t0;
                    float denom = det(t01, t02);
                    float u = det(p, t02) / denom;
                    float v = det(t01, p) / denom;
                    cv::Vec3f vert = u*v1+v*v2+(1-u-v)*v0;
                    texel2coords[y*w+x].push_back(vert);
                }
            }
        }
    }
}

bool TextureLT::visible(const cv::Vec3f& p1, const cv::Vec3f& p2)
{
    RTCRay shadow;
    shadow.org[0] = p1[0]; shadow.org[1] = p1[1]; shadow.org[2] = p1[2];
    cv::Vec3f dir = normalize(p2 - p1);
    shadow.dir[0] = dir[0]; shadow.dir[1] = dir[1]; shadow.dir[2] = dir[2];
    shadow.tnear = epsilon;
    shadow.tfar = norm(p2 - p1) - epsilon;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;
    /* trace shadow ray */
    rtcOccluded(scene, shadow);
    return shadow.geomID == RTC_INVALID_GEOMETRY_ID;
}

void TextureLT::get_light_transport_range(cv::Mat1f& lt, const cv::Vec3f& org, const cv::Vec3f& dir, int start, int end)
{
    for(int index = start; index < end; index++)
    {
        bool vis = false;
        cv::Vec3f d(0, 1, 0);
        for(int k=0; k<texel2coords[index].size(); k++)
        {
            vis = visible(org, texel2coords[index][k]);
            d = normalize(texel2coords[index][k] - org);
        }
        lt.at<float>(index) = vis * powf(fabs(d.dot(dir)), 1.0f);
    }
}

cv::Mat1f TextureLT::get_light_transport(const cv::Vec3f& org, const cv::Vec3f& dir)
{
    const unsigned& w = width;
    unsigned h = texel2coords.size() / w;
    cv::Mat1f lt(w, h);
    lt.setTo(0);
    const int chunk_size = 512;
    
    std::vector<LightTransportTask> tasks;
    for(int i=0; i<texel2coords.size()/chunk_size; i++)
    {
        LightTransportTask task;
        task.start = i*chunk_size;
        task.end = (i+1)*chunk_size;
        task.lt = &lt;
        task.org = org;
        task.dir = dir;
        tasks.push_back(task);
    }
    
    Executor exec(tasks);
    tbb::parallel_for(tbb::blocked_range<size_t>(0,tasks.size()),exec);
    return lt;
}

void TextureLT::cleanup()
{
    if(scene) rtcDeleteScene(scene);
    scene = NULL;
    if(device) rtcDeleteDevice(device);
    device = NULL;
}
