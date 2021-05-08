#ifndef gridmap_painter_H
#define gridmap_painter_H

#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>

#include "map_tf/map_tf.h"
   
class Painter{
public:

    struct GridMap{
        std::vector<int> data;
        double grid_size;
        int grid_size_p;
        int raw; // 行 x軸(width)
        int col; // 列 y軸(height)
    };

    Painter();
    Painter(double grid_size);
    ~Painter();
    void timerCallback(const ros::TimerEvent&);
    void printMapInfo();
    void updateGrid(double x, double y, int value);
    void paintAllGrid();
    void saveImage(std::string path);
    void paintFootmark();
    void getNowPoint(geometry_msgs::Pose2D& pose);

    std::vector<cv::Scalar> colors;
   
private:

    int pointToGrid(double x, double y);
    void createGridMap(double grid_size);
    void paintGrid(int index, int value);

    ros::Timer timer_;
    mapTF map_;
    GridMap grid_map_;
    std::vector<mapTF::Point2D> footmarks_;

};

#endif