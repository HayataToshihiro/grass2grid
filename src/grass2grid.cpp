#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>

sensor_msgs::Image seg;
sensor_msgs::Image depth;
nav_msgs::OccupancyGrid map;

void seg_callback(const sensor_msgs::ImageConstPtr& msg)
{
    seg = *msg;//grassのピクセルの取得？
}

void depth_callback(const sensor_msgs::ImageConstPtr& msg)
{
    depth = *msg;//grassのピクセル座標とdepthからgrassの座標を計算？
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grass2grid");
    ros::NodeHandle nh;

    ros::Subscriber seg_sub = nh.subscribe("/deeplab/image", 100, seg_callback);
    ros::Subscriber depth_sub = nh.subscribe("/camera/deapth/resized_image", 100, depth_callback);

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 100, true);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        printf("----\n");






        
        map_pub.publish(map);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
