#include "gridmap_painter/gridmap_painter.h"
#include "kbhit.h"
#include "getch.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gridmap_painter");
    //ノード名の初期化
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(5);

    Painter a(0.2); // mapを0.2mの大きさのグリッドマップにする (地図のグリッドマップ)
    ros::Duration(1).sleep();

    geometry_msgs::Pose2D robot_pose;       // ロボットの現在位置姿勢(/map)
    geometry_msgs::Pose2D step_pose_local;  // 段差の位置(/base_link)
    geometry_msgs::Pose2D step_pose_global; // 段差の位置(/map)

    while(ros::ok()){
        a.getNowPoint(robot_pose);    // 現在位置姿勢の取得(/map上の/base_link)
        a.printMapInfo();       // 位置情報の出力

        /* 
         ここでLiDARから得られたデータを用いて
         base_link座標のグリッドマップ(ロボットと一緒に移動する)の
         各グリッド処理を行う.
         その結果例として以下の座標付近のグリッドの段差情報が更新された
         (0.5, 0.5), (0.5, -0.2).../base_link
        */
        step_pose_local.x = 0.5;
        step_pose_local.y = 0.5;
        step_pose_global.x = step_pose_local.x * cos(robot_pose.theta) - step_pose_local.y * sin(robot_pose.theta) + robot_pose.x;
        step_pose_global.y = step_pose_local.x * sin(robot_pose.theta) + step_pose_local.y * cos(robot_pose.theta) + robot_pose.y;
        a.updateGrid(step_pose_global.x, step_pose_global.y, 0); // map座標が含まれるグリッドを値0に更新

        step_pose_local.x = 0.5;
        step_pose_local.y = -0.2;
        step_pose_global.x = step_pose_local.x * cos(robot_pose.theta) - step_pose_local.y * sin(robot_pose.theta) + robot_pose.x;
        step_pose_global.y = step_pose_local.x * sin(robot_pose.theta) + step_pose_local.y * cos(robot_pose.theta) + robot_pose.y;
        a.updateGrid(step_pose_global.x, step_pose_global.y, 1); // map座標が含まれるグリッドを値1に更新

        if (kbhit() && getche() == 's'){
            // sキーが押されたら保存
            a.paintAllGrid();   // グリッド情報を地図画像に反映
            a.paintFootmark();  // 通過箇所を線で描画
            a.saveImage("/home/suzuki-t/sample.png");   // その地図画像を保存
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}