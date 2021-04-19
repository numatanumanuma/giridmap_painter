#ifndef gridmap_painter_H
#define gridmap_painter_H

#include "ros/ros.h"
#include "map_tf/map_tf.h"
   
class Painter{
public:

    struct GridMap{
        std::vector<int> grid_map;
        double grid_size;
    };

    Painter();
    ~Painter();
    void timerCallback(const ros::TimerEvent&);
    void printMapInfo();
    void paintGrid();
    void saveImage(std::string path);
   
private:

    ros::Timer timer_;
    mapTF map_;

};

#endif