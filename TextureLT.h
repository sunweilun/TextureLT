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
    static RTCDevice device;
    static RTCScene scene;
    static float epsilon;
    static std::vector<tinyobj::shape_t> shapes;
    static std::vector<tinyobj::material_t> materials;
    static std::vector<std::vector<cv::Vec3f> > texel2coords;
    static unsigned width;
    static bool visible(const cv::Vec3f& p1, const cv::Vec3f& p2);
    static void get_light_transport_range(cv::Mat1f& lt, const cv::Vec3f& org, const cv::Vec3f& dir, int start, int end);
public:
    static void init();
    static void load_scene(const std::string& path);
    static void fill_texel2coord(const std::string& shape_name, int w, int h);
    static cv::Mat1f get_light_transport(const cv::Vec3f& org, const cv::Vec3f& dir);
    static void cleanup();
};

struct LightTransportTask 
{
    void operator()() 
    {
        TextureLT::get_light_transport_range(*lt, org, dir, start, end);
    }
    int start;
    int end;
    cv::Mat1f* lt;
    cv::Vec3f org;
    cv::Vec3f dir;
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

