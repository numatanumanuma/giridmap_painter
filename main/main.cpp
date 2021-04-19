#include "gridmap_painter/gridmap_painter.h"
#include "kbhit.h"
#include "getch.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gridmap_painter");
    //ノード名の初期化

    Painter a;
    ros::Rate loop_rate(10);

    while(ros::ok()){
        if (kbhit() && getche() == 's'){
            a.printMapInfo();
            a.saveImage("/home/suzuki-t/sample.png");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}