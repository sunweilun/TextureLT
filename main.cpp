/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: swl
 *
 * Created on June 10, 2016, 11:08 AM
 */



#include <sys/time.h>
#include <stdio.h>
#include "TextureLT.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv)
{
    TextureLT::init();
    TextureLT::load_scene("models/planes.obj");
    TextureLT::fill_texel2sp("wall", 512, 512);
    cv::namedWindow("image");
    float theta = 0;
    float phi = M_PI / 3.0;
    while(1)
    {
        cv::Vec3f org(sin(phi)*cos(theta), cos(phi), sin(phi)*sin(theta));
        struct timeval ts, te;
        gettimeofday(&ts, NULL);
        cv::Mat4f lt = TextureLT::get_light_transport(org*5.f, -org, cv::Vec3f(0, 1, 0));
        
        cv::imshow("image", lt);
        int key = cv::waitKey(10);
        if(key == 27) break;
        if(key == 'w'){ phi -= 0.05; }
        if(key == 's'){ phi += 0.05; }
        if(key == 'a'){ theta -= 0.05; }
        if(key == 'd'){ theta += 0.05; }
        
        gettimeofday(&te, NULL);
        //printf("%lfus\n", (te.tv_sec - ts.tv_sec)*1e6+te.tv_usec-ts.tv_usec);
    }
    TextureLT::cleanup();
    return 0;
}

