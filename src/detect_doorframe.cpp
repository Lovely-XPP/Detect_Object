#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/types_c.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <px4_cmd/Command.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <px4_cmd/template/single_vehicle_external_command.hpp>

using namespace std;
using namespace cv;

#define PI 3.14159265358979323846

typedef struct detect_information
{
    double ex = 0;
    double ey = 0;
    int w = 0;
    int flag = 0;
} Detect_Info;
Detect_Info detect_info;
bool simulation = true;
string camera_topic = "/iris/usb_cam/image_raw";
cv::Mat image_from_topic;
bool shutdown_flag = false;

ros::Subscriber image_sub;

void redDetectHSV();
void send_ext_cmd();
void image_raw_sub(const sensor_msgs::Image::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_ballon");
    ros::NodeHandle nh("~");

    nh.getParam("simulation", simulation);
    if (simulation)
    {
        if (!nh.getParam("camera_topic", camera_topic))
        {
            ROS_ERROR("Camera Topic Not Found! Programme Exit...");
            return 0;
        }
        else
        {
            ROS_INFO_STREAM("Input Camera Topic: " << camera_topic);
        }
    }
    image_sub = nh.subscribe<sensor_msgs::Image>(camera_topic, 20, image_raw_sub);

    while (simulation && image_sub.getNumPublishers() < 1)
    {
        ROS_WARN_STREAM("Recive_Camera: Can not Connet to Camera Topic [" << camera_topic << "] , Retrying ...");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Recive_Camera: Connect to Camera Topic [" << camera_topic << "] Successfully!");
    std::thread ros_thread(send_ext_cmd);
    ros_thread.detach();

    redDetectHSV();
    shutdown_flag = true;
    sleep(1);

    nh.shutdown();
    return 0;
}

void send_ext_cmd()
{
    single_vehicle_external_command ext_cmd;
    ext_cmd.total_time = -1;
    ext_cmd.start();
    double minz = 1;
    float q0 = 0, q1 = 0, q2 = 0, q3 = 0;
    int reverse_flag;
    double desire_cmd_value[3];
    bool init = true;
    int fail_count = 0;
    double last_pos = 0;
    double yaw_value = 0;
    float fly_init_time = 20;
    float fly_time = fly_init_time;
    bool fly_across_circle_flag = false;
    cv::Scalar HSV_lower = {66, 91, 51};
    cv::Scalar HSV_upper = {255, 255, 255};
    while (ros::ok() && !shutdown_flag)
    {
        while (!ext_cmd.ext_on)
        {
            init = true;
            ROS_INFO("External Command: Waiting for user-define mode!");
            ros::Duration(0.5).sleep();
            if (shutdown_flag)
            {
                return;
            }
        }
        if (init)
        {
            yaw_value = 0 - PI / 2;
            init = false;
        }
        if (detect_info.flag == 2 || fly_across_circle_flag)
        {
            if (fly_time > fly_init_time / 2)
            {
                desire_cmd_value[0] = 0;
                //"Y Velocity [m/s] 向左
                desire_cmd_value[1] = 0.005 * detect_info.ex; // ex = 320-cx
                //"Z Velocity [m/s] 向上
                desire_cmd_value[2] = 0.01 * (detect_info.ey); // ey = 240-cy
                ext_cmd.set_velocity(desire_cmd_value[0], desire_cmd_value[1], desire_cmd_value[2], yaw_value, px4_cmd::Command::BODY);
            }
            else
            {
                desire_cmd_value[0] = 0.5;
                desire_cmd_value[1] = 0;
                desire_cmd_value[2] = 0;
                ext_cmd.set_velocity(desire_cmd_value[0], desire_cmd_value[1], desire_cmd_value[2], yaw_value, px4_cmd::Command::BODY);
            }
            
            fly_across_circle_flag = true;
            fly_time -= 1;
            if (fly_time < 0)
            {
                //fly_time = fly_init_time;
                //fly_across_circle_flag = false;
                ROS_INFO("Flying Accross DoorFrame Done. Changing to Hover Mode.");
                ext_cmd.set_hover(yaw_value);
                break;
            }
            ROS_INFO("Flying Accross DoorFrame...");
            ros::Duration(1).sleep();
            continue;
        }
        if (detect_info.flag == 1 && abs(detect_info.ex) < 120)
        {
            //"X Velocity [m/s] 向前
            if (sqrt(detect_info.ex * detect_info.ex + detect_info.ey * detect_info.ey) < 10)
            {
                desire_cmd_value[0] = 0.005 * (250 - detect_info.w);
            }
            else
            {
                desire_cmd_value[0] = 0;
            }
            // desire_cmd_value[0] = 0;

            //"Y Velocity [m/s] 向左
            desire_cmd_value[1] = 0.005 * detect_info.ex; // ex = 320-cx
            //"Z Velocity [m/s] 向上
            desire_cmd_value[2] = 0.01 * (detect_info.ey); // ey = 240-cy
                
            fail_count = 0; 
            last_pos = detect_info.ex / abs(detect_info.ex);
            /*
            desire_cmd_value1[0] = desire_cmd_value1[0] + pid_w.pid_control(250, detect_info.w);//pid_control(desire, actual)
            desire_cmd_value1[1] = desire_cmd_value1[1] + pid_ey.pid_control(0, detect_info.ey);//pid_control(desire, actual)
            desire_cmd_value1[2] = desire_cmd_value1[2] + pid_ex.pid_control(0, detect_info.ex);//pid_control(desire, actual)

            cout << "pid_value:" << desire_cmd_value1[0] << " " << desire_cmd_value1[1]  <<  "  " << desire_cmd_value1[2] << endl;
            */

            // 速度限幅
            if (desire_cmd_value[0] > 0.3)
            {
                desire_cmd_value[0] = 0.3;
            }
            if (desire_cmd_value[0] < -0.3)
            {
                desire_cmd_value[0] = -0.3;
            }
            if (desire_cmd_value[1] > 0.3)
            {
                desire_cmd_value[1] = 0.3;
            }
            if (desire_cmd_value[1] < -0.3)
            {
                desire_cmd_value[1] = -0.3;
            }
            if (desire_cmd_value[2] > 0.3)
            {
                desire_cmd_value[2] = 0.3;
            }
            if (desire_cmd_value[2] < -0.3)
            {
                desire_cmd_value[2] = -0.3;
            }

            ROS_INFO("Detect DoorFarme!");
        }
        else
        {
            if (detect_info.flag == 0)
            {
                fail_count += 1;
                if (fail_count > 5)
                {
                    desire_cmd_value[0] = 0;
                    desire_cmd_value[1] = 0;
                    desire_cmd_value[2] = 0.01;

                    ROS_WARN("Not Detect DoorFrame!");
                    if (last_pos > 0) // ex > 0 框在左边，飞机在右边
                    {
                        yaw_value -= PI / 60;
                    }
                    else if (last_pos < 0) // ex > 0 框在右边，飞机在左边
                    {
                        yaw_value += PI / 60;
                    }
                    else
                    {
                        if (reverse_flag == 0)
                        {
                            yaw_value += PI / 60;
                        }
                        if (reverse_flag == 1)
                        {
                            yaw_value -= PI / 60;
                        }
                    }
                }
            }
        } // end else
        if (abs(yaw_value) > 2 * PI)
        {
            yaw_value = yaw_value - floor(yaw_value / 2 / PI) * 2 * PI;
        }
        ext_cmd.set_velocity(desire_cmd_value[0], desire_cmd_value[1], desire_cmd_value[2], yaw_value, px4_cmd::Command::BODY);
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    ext_cmd.shutdown();
}

// 白平衡-完美反射
cv::Mat WhiteBalance_PRA(cv::Mat src)
{
    cv::Mat result = src.clone();
    if (src.channels() != 3)
    {
        ROS_WARN("The number of image channels is not 3.");
        return result;
    }

    // 通道分离
    vector<cv::Mat> Channel;
    cv::split(src, Channel);

    // 定义参数
    int row = src.rows;
    int col = src.cols;
    int RGBSum[765] = {0};
    uchar maxR, maxG, maxB;

    // 计算单通道最大值
    for (int i = 0; i < row; ++i)
    {
        uchar *b = Channel[0].ptr<uchar>(i);
        uchar *g = Channel[1].ptr<uchar>(i);
        uchar *r = Channel[2].ptr<uchar>(i);
        for (int j = 0; j < col; ++j)
        {
            int sum = b[j] + g[j] + r[j];
            RGBSum[sum]++;
            maxB = max(maxB, b[j]);
            maxG = max(maxG, g[j]);
            maxR = max(maxR, r[j]);
        }
    }

    // 计算最亮区间下限T
    int T = 0;
    int num = 0;
    int K = static_cast<int>(row * col * 0.5);
    for (int i = 765; i >= 0; --i)
    {
        num += RGBSum[i];
        if (num > K)
        {
            T = i;
            break;
        }
    }

    // 计算单通道亮区平均值
    double Bm = 0.0, Gm = 0.0, Rm = 0.0;
    int count = 0;
    for (int i = 0; i < row; ++i)
    {
        uchar *b = Channel[0].ptr<uchar>(i);
        uchar *g = Channel[1].ptr<uchar>(i);
        uchar *r = Channel[2].ptr<uchar>(i);
        for (int j = 0; j < col; ++j)
        {
            int sum = b[j] + g[j] + r[j];
            if (sum > T)
            {
                Bm += b[j];
                Gm += g[j];
                Rm += r[j];
                count++;
            }
        }
    }
    Bm /= count;
    Gm /= count;
    Rm /= count;

    // 通道调整
    Channel[0] *= maxB / Bm;
    Channel[1] *= maxG / Gm;
    Channel[2] *= maxR / Rm;

    // 合并通道
    cv::merge(Channel, result);

    if (count == 0)
    {
        cv::Mat fail(1, 1, CV_8UC3);
        return cv::Mat();
    }

    return result;
}

cv::Mat WhiteBalance_Gray(cv::Mat src)
{
    cv::Mat result = src.clone();
    if (src.channels() != 3)
    {
        ROS_WARN("The number of image channels is not 3.");
        return result;
    }

    // 通道分离
    std::vector<cv::Mat> Channel;
    cv::split(src, Channel);

    // 计算通道灰度值均值
    double Bm = cv::mean(Channel[0])[0];
    double Gm = cv::mean(Channel[1])[0];
    double Rm = cv::mean(Channel[2])[0];
    double Km = (Bm + Gm + Rm) / 4;

    // 通道灰度值调整
    Channel[0] *= Km / Bm;
    Channel[1] *= Km / Gm;
    Channel[2] *= Km / Rm;

    // 合并通道
    cv::merge(Channel, result);

    return result;
}

void redDetectHSV()
{
    Mat imgOriginal, imgBalance, imgHSV, imgRGB, imgGray, output;
    Mat element1, element2;
    Mat HSV_mask;
    Mat thresh;
    Mat connect_lables;
    Mat connect_output;
    Mat connect_stats;
    Mat connect_centroids;
    vector<vector<Point>> contours;
    std::vector<cv::Vec3f> circles;
    cv::Scalar HSV_lower = cv::Scalar(66, 91, 51);
    cv::Scalar HSV_upper = cv::Scalar(255, 255, 255);
    double minRadius = 40;
    double maxRadius = 120;
    while (true)
    {
        if (!simulation)
        {
            VideoCapture cap(0);
            cap.read(imgOriginal);
        }
        else
        {
            ros::spinOnce();
            imgOriginal = image_from_topic.clone();
        }
        imgBalance = WhiteBalance_PRA(imgOriginal);
        if (imgBalance.rows <= 1)
        {
            imgBalance = WhiteBalance_Gray(imgOriginal);
        }
        imgRGB = imgBalance.clone();

        // 因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
        cvtColor(imgRGB, imgHSV, COLOR_BGR2HSV); // Convert the captured frame from BGR to HSV
        inRange(imgHSV, HSV_lower, HSV_upper, HSV_mask);
        int n = connectedComponentsWithStats(HSV_mask, connect_lables, connect_stats, connect_centroids);
        int max_label_id = 0, max_area = 0;
        imgGray = cv::Mat::zeros(imgOriginal.size(), CV_8UC1);

        if (n != 1)
        {
            for (size_t i = 1; i < n; i++)
            {
                if (connect_stats.at<int>(i, 4) > max_area)
                {
                    max_area = connect_stats.at<int>(i, 4);
                    max_label_id = i;
                }
            }
            
            for (int y = 0; y < imgGray.rows; y++)
            {
                for (int x = 0; x < imgGray.cols; x++)
                {
                    int label = connect_lables.at<int>(y, x);
                    CV_Assert(0 <= label && label <= n);
                    if (label == max_label_id)
                    {
                        imgGray.at<int8_t>(y, x) = 255;
                    }
                }
            }
        }
        findContours(imgGray, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        cv::imshow("gray", imgGray); // show the gray image
        threshold(imgGray, output, 120, 255, THRESH_BINARY_INV);

        circles.clear();
        float area = 0, arc_length = 0, radius = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {
            if(contours[i].size() < 5)
            {
                continue;
            }

            area = contourArea(contours[i]);

            if (area < minRadius * minRadius * PI || area > maxRadius * maxRadius * PI)
            {
                continue;
            }

            arc_length = arcLength(contours[i], true);
            radius = arc_length / PI / 2;
            if (radius < minRadius || radius > maxRadius)
            {
                continue;
            }

            cv::RotatedRect circle = fitEllipse(contours[i]);
            circles.push_back(Vec3f(circle.center.x, circle.center.y, radius));
        }

        float _max = 0;
        int _maxid = 0;
        if (!circles.empty()) // 如果有检测结果， 取最大值
        {
            int k = 0;
            for (size_t i = 0; i < circles.size(); i++)
            {
                cv::Vec3f cc = circles[i];
                // cc[0] -> x  cc[1] -> y  cc[2] -> r
                cv::circle(imgOriginal, cv::Point(cc[0], cc[1]), cc[2], cv::Scalar(0, 0, 255), 2, cv::LINE_AA); // 可视化圆弧
                cv::circle(imgOriginal, cv::Point(cc[0], cc[1]), 1, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);     // 可视化圆心
                if (cc[2] > _max)
                {
                    _max = cc[2];
                    _maxid = i;
                }
            }

            if (_max > 30)
            {
                ROS_INFO("Detect Result: [%.2f, %.2f, %.2f, %.2f]", circles[_maxid][0], circles[_maxid][1], circles[_maxid][2], circles[_maxid][2] / min(imgGray.rows, imgGray.cols));
                detect_info.ex = imgOriginal.cols / 2 - circles[_maxid][0];
                detect_info.ey = imgOriginal.rows / 2 - circles[_maxid][1];
                detect_info.w = circles[_maxid][2];
                detect_info.flag = 1;
                // 如果半径很大（占图像的1/3），那么离门框很近了，这个时候则为穿越模式
                if (circles[_maxid][2] > min(imgGray.rows, imgGray.cols) / 3 && (sqrt(detect_info.ex * detect_info.ex + detect_info.ey * detect_info.ey) < 10))
                {
                    detect_info.flag = 2;
                }
            }
            else // 检测到轮廓，但是不符合气球约束
            {
                detect_info.ex = 0;
                detect_info.ey = 0;
                detect_info.w = 250;
                detect_info.flag = 0;
            }

        } // end if
        else
        { // 没有检测结果
            detect_info.ex = 0;
            detect_info.ey = 0;
            detect_info.w = 250;
            detect_info.flag = 0;
        }
        cv::imshow("raw_img", imgOriginal);
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }
    destroyAllWindows();
} // end main func

void image_raw_sub(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::Image current_state = *msg;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(cv_ptr->image, image_from_topic, COLOR_RGB2BGR);
}
