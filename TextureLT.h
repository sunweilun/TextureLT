/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TextureLT.h
 * Author: swl
 *
 * Created on June 10, 2016, 11:44 AM
 */

#ifndef TEXTURELT_H
#define TEXTURELT_H

#include <cstdlib>
#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>
#include <xmmintrin.h>
#include <pmmintrin.h>
#include "tiny_obj_loader.h"
#include <opencv2/opencv.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>

struct LightTransportTask;

class TextureLT 
{
    friend struct LightTransportTask;
protected:
    struct SurfacePoint
    {
        cv::Vec3f position;
        cv::Vec3f normal;
        unsigned matID;
        float shade(const cv::Vec3f& wi, const cv::Vec3f& wo) { return 0; }
        SurfacePoint(unsigned geomID, unsigned primID, float u, float v)
        {
            const tinyobj::mesh_t& mesh = shapes[geomID].mesh;
            unsigned i0 = mesh.indices[3*primID];
            unsigned i1 = mesh.indices[3*primID+1];
            unsigned i2 = mesh.indices[3*primID+2];
            cv::Vec3f v0 = ((cv::Vec3f*)mesh.positions.data())[i0];
            cv::Vec3f v1 = ((cv::Vec3f*)mesh.positions.data())[i1];
            cv::Vec3f v2 = ((cv::Vec3f*)mesh.positions.data())[i2];
            position = u*v1+v*v2+(1-u-v)*v0;
            cv::Vec3f cr = (v1-v0).cross(v2-v1);
            normal = normalize(cr);
            if(mesh.normals.size())
            {
                cv::Vec3f n0 = ((cv::Vec3f*)mesh.normals.data())[i0];
                cv::Vec3f n1 = ((cv::Vec3f*)mesh.normals.data())[i1];
                cv::Vec3f n2 = ((cv::Vec3f*)mesh.normals.data())[i2];
                normal = normalize(u*n1+v*n2+(1-u-v)*n0);
            }
            matID = -1;
            if(mesh.material_ids.size())
            {
                matID = mesh.material_ids[primID];
            }
        }
        float diffComp(const cv::Vec3f& wi, const cv::Vec3f& wo) const
        {
            float d = -normal.dot(wi);
            if(d<0) return 0;
            return d;
        }
        float specComp(const cv::Vec3f& wi, const cv::Vec3f& wo) const
        {
            cv::Vec3f refl = wo - 2*normal.dot(wo)*normal;
            float nd = -normal.dot(wi);
            float d = refl.dot(wi);
            if (nd <= 0 || d <= 0) return 0;
            float s = 50.f;
            if(matID < materials.size())
                s = materials[matID].shininess;
            return powf(d, s) * nd;
        }
        cv::Vec3f getDiffColor(const cv::Vec3f& wi, const cv::Vec3f& wo) const
        {
            return cv::Vec3f(0.f);
            if(matID < materials.size())
                return diffComp(wi, wo) * cv::Vec3f(materials[matID].diffuse);
        }
        cv::Vec3f getSpecColor(const cv::Vec3f& wi, const cv::Vec3f& wo) const
        {
            return cv::Vec3f(0.f);
            if(matID < materials.size())
                return specComp(wi, wo) * cv::Vec3f(materials[matID].specular);
        }
        
    };
    
    static RTCDevice device;
    static RTCScene scene;
    static float epsilon;
    static std::vector<tinyobj::shape_t> shapes;
    static std::vector<tinyobj::material_t> materials;
    static std::vector<std::vector<SurfacePoint> > texel2sp;
    static unsigned width;
    static bool visible(const cv::Vec3f& p1, const cv::Vec3f& p2);
    static void get_light_transport_range(cv::Mat4f& lt, 
        const cv::Vec3f& org, const cv::Vec3f& dir, int start, int end, 
        const SurfacePoint& sp, const cv::Vec3f& ls);
public:
    static void init();
    static void load_scene(const std::string& path);
    static void fill_texel2sp(const std::string& shape_name, int w, int h);
    static cv::Mat4f get_light_transport(const cv::Vec3f& org, const cv::Vec3f& dir, const cv::Vec3f& ls);
    static void cleanup();
};

struct LightTransportTask 
{
    void operator()() 
    {
        TextureLT::get_light_transport_range(*lt, org, dir, start, end, *sp, ls);
    }
    int start;
    int end;
    cv::Mat4f* lt;
    cv::Vec3f org;
    cv::Vec3f dir;
    TextureLT::SurfacePoint* sp;
    cv::Vec3f ls; // light source position
};

struct Executor 
{
    Executor(std::vector<LightTransportTask>& t)
    : _tasks(t) { }

    Executor(Executor& e, tbb::split)
    : _tasks(e._tasks) { }

    void operator()(const tbb::blocked_range<size_t>& r) const 
    {
        for (size_t i = r.begin(); i != r.end(); ++i)
            _tasks[i]();
    }

    std::vector<LightTransportTask>& _tasks;
};

#endif /* TEXTURELT_H */

