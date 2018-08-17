//
// Created by hd on 1/10/18.
//

#ifndef PROJECT_UTIL_H
#define PROJECT_UTIL_H
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <ostream>
#include <iostream>
#include <string>
#include <ctime>
#include <ros/ros.h>
#include <chrono>

const struct SensorType
{
    SensorType() {}

    unsigned char both = 0;
    unsigned char wide_camera = 1;
    unsigned char long_camera = 2;
    unsigned char imu = 3;
    unsigned char heading = 4;
    unsigned char other = 5;
}sensor_type;

using namespace Eigen;
using namespace std::chrono;

namespace Util{

    class Timer
    {
    public:
        Timer()
        {
            tic();
        }
        double end()
        {
            duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
            start = std::clock();
            return duration;
        }
        void tic()
        {
            start = std::clock();
        }
        void toc(std::string section = " ")
        {
            end();
            printf("%s T: %g FPS: %gHz \r\n", section.c_str(), duration, 1/duration);
        }
        void hz(std::string section = " ")
        {
            ++num;
            duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

            if (duration >= 1.0)
            {
                printf("%s: %f Hz\n", section.c_str(), double(num)/duration);
                num = 0;
                tic();
            }
        }
    private:
        std::clock_t start;
        double duration;
        unsigned int num;
    };

    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };

    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };

    class ROSTimer
    {
    public:
        ROSTimer()
        {
            ros::Time::init();
            tic();
        }
        double end()
        {
            duration = ros::Time::now().toSec() - start;
            start = ros::Time::now().toSec();
            return duration;
        }
        void tic()
        {
            start = ros::Time::now().toSec();
        }
        void toc(std::string section = " ")
        {
            end();
            printf("%s T: %g FPS: %gHz \r\n", section.c_str(), duration, 1/duration);
        }
        void hz(std::string section = " ")
        {
            ++num;
            duration = ros::Time::now().toSec() - start;

            if (duration >= 1.0)
            {
                printf("%s: %f Hz\n", section.c_str(), double(num)/duration);
                num = 0;
                tic();
            }
        }
    private:
        double start;
        double duration;
        unsigned int num;
    };

    class CPPTimer
    {
    public:
        CPPTimer()
        {
            tic();
        }
        double end()
        {
            time_span = duration_cast<duration<double>> (high_resolution_clock::now() - start);
            tic();
            return time_span.count();
        }
        void tic()
        {
            start = high_resolution_clock::now();
        }
        void toc(std::string section = " ")
        {
            end();
            printf("%s T: %g FPS: %gHz \r\n", section.c_str(), time_span.count(), 1/(time_span.count()));
        }
        void hz(std::string section = " ")
        {
            ++num;
            time_span = duration_cast<duration<double>> (high_resolution_clock::now() - start);

            if (time_span.count() >= 1.0)
            {
                printf("%s: %f Hz\n", section.c_str(), double(num)/(time_span.count()));
                num = 0;
                tic();
            }
        }
    private:
        high_resolution_clock::time_point start;
        duration<double> time_span;
        unsigned int num;
    };

}

#endif //PROJECT_UTIL_H
