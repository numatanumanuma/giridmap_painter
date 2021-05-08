#include "gridmap_painter/gridmap_painter.h"
   
Painter::Painter(){
    ros::NodeHandle nh("~");
    timer_ = nh.createTimer(ros::Duration(0.5), &Painter::timerCallback, this);
    while (!map_.get_map){
        std::cout << "wait..." << std::endl;
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    createGridMap(0.2);
    // 初期色
    colors = {cv::Scalar(0, 255, 0), cv::Scalar(0, 150, 0), cv::Scalar(0, 50, 0)};
}

Painter::Painter(double grid_size){
    ros::NodeHandle nh("~");
    timer_ = nh.createTimer(ros::Duration(0.5), &Painter::timerCallback, this);
    while (!map_.get_map){
        std::cout << "wait..." << std::endl;
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    createGridMap(grid_size);
    colors = {cv::Scalar(0, 255, 0), cv::Scalar(0, 150, 0), cv::Scalar(0, 50, 0)};
}

Painter::~Painter(){
    
}

void Painter::timerCallback(const ros::TimerEvent&){
    
}

/*
 引数はグリッドの大きさ(m)
*/
void Painter::createGridMap(double grid_size){
    grid_map_.grid_size     = grid_size;
    grid_map_.grid_size_p   = grid_size / map_.resolution;
    grid_map_.raw   = map_.width / grid_map_.grid_size_p;
    grid_map_.col   = map_.height / grid_map_.grid_size_p;
    grid_map_.data  = std::vector<int>(grid_map_.raw * grid_map_.col, -1);
    std::cout << "created grid map" << std::endl;
    std::cout << "raw : " << grid_map_.raw << ", col : " << grid_map_.col << std::endl;
}

/*
 各グリッドの値を更新
 /mapで受け取る
*/
void Painter::updateGrid(double x, double y, int value){
    int index = pointToGrid(x, y);
    grid_map_.data[index] = value;
}

/*
 グリッドの情報を地図画像に反映
*/
void Painter::paintAllGrid(){
    for(int i; i < grid_map_.data.size(); i++){
        int value = grid_map_.data[i];
        if (value >= 0){
            // この先でも弾いているのでこのifはあまり意味ない
            paintGrid(i, value);
        }
    }
    cv::waitKey(1);
}

void Painter::saveImage(std::string path){
    cv::imwrite(path, map_.image);
    ROS_INFO("image saved!!");
}

/*
 通った場所に線を塗る
 通ってきた場所はfootmarks_に保存してある
*/
void Painter::paintFootmark(){

    for (int i = 1; i < footmarks_.size(); i++){
        cv::line(map_.image, 
            cv::Point(footmarks_[i-1].x_p,  footmarks_[i-1].y_p ), 
            cv::Point(footmarks_[i].x_p,    footmarks_[i].y_p ), 
            cv::Scalar(0, 0, 255), 2, cv::LINE_4
            );
    }
    cv::waitKey(1);
}

/*
 map_tfのgetNowPointはmapTF::Point2Dで返してくるのでgeometry_msgs::Pose2Dにして返す
*/
void Painter::getNowPoint(geometry_msgs::Pose2D& pose){
    mapTF::Point2D p;
    map_.getNowPoint(p);
    pose.x = p.x;
    pose.y = p.y;
    pose.theta = p.yaw;
    footmarks_.push_back(p); // ここで足跡を保存している
}

// private //

void Painter::paintGrid(int index, int value){
    if (value < 0 || value >= colors.size())
        return;
    int x = index % grid_map_.raw;
    int y = index / grid_map_.raw;
    int lu_x = x * grid_map_.grid_size_p;
    int lu_y = y * grid_map_.grid_size_p;
    cv::Point p1(lu_x, lu_y), p2(lu_x + grid_map_.grid_size_p, lu_y + grid_map_.grid_size_p);
	cv::rectangle(map_.image, p1, p2, colors[value], -1, cv::LINE_4);
}

void Painter::printMapInfo(){
    map_.printNowPoint();
}

/*
 map座標からどのグリッドに含まれるかを計算 
 grid_map_のindexを返す
*/
int Painter::pointToGrid(double x, double y){
    mapTF::Point2D p;
    p.x = x;
    p.y = y;
    map_.pointToPixel(p);
    int raw = p.x_p / grid_map_.grid_size_p;
    int col = p.y_p / grid_map_.grid_size_p;
    return col * grid_map_.raw + raw;
}
