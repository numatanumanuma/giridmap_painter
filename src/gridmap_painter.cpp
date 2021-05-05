#include "gridmap_painter/gridmap_painter.h"
   
Painter::Painter(){
    ros::NodeHandle nh("~");
    timer_ = nh.createTimer(ros::Duration(0.5), &Painter::timerCallback, this);
    if (!map_.get_map){
        ros::Duration(0.1).sleep();
    }
}

Painter::~Painter(){
    
}

void Painter::timerCallback(const ros::TimerEvent&){
    printMapInfo();
    paintGrid();
}

void Painter::paintGrid(){
    int r, g, b;
    r = 255;
    g = 70;
    b = 0;
    mapTF::Point2D p;
    map_.getNowPoint(p);
    cv::circle(map_.image, cv::Point(p.x_p, p.y_p), 5, cv::Scalar(b, g, r), 2, 4);
    // map_.dot_image(cv::Point(p.x_p, p.y_p)) = cv::Vec3d(b, g, r);
}

void Painter::printMapInfo(){
    map_.printNowPoint();
}

void Painter::saveImage(std::string path){
    cv::imwrite(path, map_.image);
    ROS_INFO("image saved!!");
}
