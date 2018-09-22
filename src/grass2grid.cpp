#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>

sensor_msgs::Image seg_img;
sensor_msgs::Image depth_img;
nav_msgs::OccupancyGrid map;

int grid[400][400] = {-1};
int seg[288][513][3] = {0};
bool seg_ok[288][513] = {false};
float depth[288][513] = {0};


cv::Mat image_s;
cv::Mat image_d;


bool im_s_s = false;
void seg_callback(const sensor_msgs::ImageConstPtr& msg)
{
    seg_img = *msg;
    image_s = cv_bridge::toCvCopy(seg_img, sensor_msgs::image_encodings::BGR8)->image;
    //resize(image_s,image_s,cv::Size(),320/513.,180/288.);
    
    cv::imshow("image_s",image_s);
    if(!im_s_s){
        cv::moveWindow("image_s",60,40);
        im_s_s = true;
    }

    //printf("image_s[160,160][B,G,R] = [%d,%d,%d]\n"
    //         ,image_s.at<cv::Vec3b>(160,160)[0],image_s.at<cv::Vec3b>(160,160)[1],image_s.at<cv::Vec3b>(160,160)[2]);
    //printf("height = %d\n",image_s.height);
    //printf("width = %d\n",image_s.width);

    for(int y=0;y<288;y++){
        for(int x=0;x<513;x++){
            for(int c=0;c<3;c++){
                seg[y][x][c] = image_s.at<cv::Vec3b>(y,x)[c];
            }
        }
    }
    for(int y=0;y<288;y++){
        for(int x=0;x<513;x++){
            if( seg[y][x][0]==0 && seg[y][x][1]==0 && seg[y][x][2]==(0||128) )seg_ok[y][x]=true;
        }
    }


    cv::waitKey(1);
    /*
    [B,G,R]=[0,0,128],[0,0,0]
    */

}

bool im_d_s = false;
void depth_callback(const sensor_msgs::ImageConstPtr& msg)
{
    depth_img = *msg;

    map.header = depth_img.header;

    image_d = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    resize(image_d,image_d,cv::Size(),513/320.,288/180.);

    cv::imshow("image_d",image_d);
    if(!im_d_s){
        cv::moveWindow("image_d",640,40);
        im_d_s = true;
    }

    //printf("image_d[90,160] = %f\n",image_d.at<float>(90,160));//(y,x)
    //printf("height = %d\n",depth_img.height);
    //printf("width = %d\n",depth_img.width);

    for(int y=0;y<288;y++){
        for(int x=0;x<513;x++){
            depth[y][x] = image_d.at<float>(y,x);
        }
    }
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grass2grid");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    ros::Subscriber seg_sub = nh.subscribe("/deeplab/image", 100, seg_callback);
    image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/resized_image", 100, depth_callback);

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 100, true);

    map.header.frame_id = "zed_depth_camera";
    map.info.resolution = 0.050000;
    map.info.width = 400;
    map.info.height = 400;
    map.info.origin.position.x = -10 ;
    map.info.origin.position.y = -10 ;
    map.info.origin.position.z = 0 ;
    map.info.origin.orientation.x = 0 ;
    map.info.origin.orientation.y = 0 ;
    map.info.origin.orientation.z = 0 ;
    map.info.origin.orientation.w = 0 ;
    map.data.resize(map.info.width * map.info.height);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        //printf("grass2grid\n");
        //ここからメイン処理（grid[][]に情報を入れる）



        //grid[][]の情報をmap.dataに入れる
        for(int y=0;y<map.info.height;y++){
            for(int x=0;x<map.info.width;x++){
                //printf("%d\n",y*map.info.width+x);
                map.data[(y*map.info.width) + x] = grid[y][x];
            }
        }

        map_pub.publish(map);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
