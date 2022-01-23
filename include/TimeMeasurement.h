#pragma once
#include <chrono>
#include <string>
#include <ros/ros.h>
#include "TimeMeasurement.h"


class TimeMeasure{
    private:
        std::chrono::system_clock::time_point start, end;
        double elapsed;
        std::string message;
    public:
        void Start(std::string title);
        void End(int display, std::string message);
        void Output();
        void Output_Ros();
        double get();
        void setMessage(std::string);
};

inline void TimeMeasure::Start(std::string title){
    start = std::chrono::system_clock::now();
    message = title;
    ROS_INFO_STREAM(title);
}
inline void TimeMeasure::End(int display = 0, std::string title = ""){
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if (title != "" ){
        setMessage(title);
    }
    ROS_INFO_STREAM(message);
    if (display) {
        std::cout << elapsed << "[ms]" << std::endl;
    } else  {
        ROS_INFO_STREAM(elapsed << "[ms]");
    }
}
inline void TimeMeasure::Output(){
    std::cout << message << ":" << elapsed << "[ms]" << std::endl;
}
inline void TimeMeasure::Output_Ros(){
    ROS_INFO_STREAM(message);
    ROS_INFO_STREAM(elapsed);
}
inline double TimeMeasure::get(){
    return elapsed;
}

inline void TimeMeasure::setMessage(std::string title = "") {
    this->message = title;
}