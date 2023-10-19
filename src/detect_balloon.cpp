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
#include <thread>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <px4_cmd/Command.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace cv;

#define PI 3.14159265358979323846

typedef struct detect_ballon_error
{
    double ex = 0;
    double ey = 0;
    int w = 0;
    int flag = 0;
} Detect_Error;
Detect_Error detect_error;
bool ext_on = false;
px4_cmd::Command cmd;
geometry_msgs::PoseStamped current_state;
bool simulation = true;
string camera_topic = "/iris/usb_cam/image_raw";
cv::Mat image_from_topic;

ros::Subscriber ext_on_sub;
ros::Subscriber state_sub;
ros::Subscriber image_sub;
ros::Publisher cmd_pub;

void redDetectHSV();
void send_ext_cmd();
void ext_on_cb(const std_msgs::Bool::ConstPtr &msg);
void image_raw_sub(const sensor_msgs::Image::ConstPtr &msg);
void state_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_ballon");
    ros::NodeHandle nh("~");

    nh.getParam("simulation", simulation);
    if (simulation)
    {
        if (!nh.getParam("camera_topic", camera_topic))
        {
            ROS_WARN("Camera Topic Not Found! Simulation Mode OFF!");
            //simulation = false;
        }
        else
        {
            ROS_INFO_STREAM("Input Camera Topic: " << camera_topic);
        }
    }
    ext_on_sub = nh.subscribe<std_msgs::Bool>("/px4_cmd/ext_on", 20, ext_on_cb);
    state_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 20, state_cb);
    image_sub = nh.subscribe<sensor_msgs::Image>(camera_topic, 20, image_raw_sub);
    cmd_pub = nh.advertise<px4_cmd::Command>("/px4_cmd/ext_command", 20);

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

    nh.shutdown();
    return 0;
}

void send_ext_cmd()
{
    cmd.ext_total_time = -1;
    cmd.ext_time = 0;
    float t = 0.0;
    double minz = 1;
    float q0 = 0, q1 = 0, q2 = 0, q3 = 0;
    int reverse_flag;
    double desire_cmd_value[3];
    bool init = true;
    int fail_count = 0;
    double last_pos = 0;
    double yaw_value = 0;
    tf::Quaternion quad;
    double pitch, roll, yaw;
    cmd.Mode = px4_cmd::Command::Move;
    cmd.Move_frame = px4_cmd::Command::BODY;
    cmd.Move_mode = px4_cmd::Command::XYZ_VEL;
    while (ros::ok())
    {
        while (!ext_on)
        {
            init = true;
            ROS_INFO("External Command: Waiting for user-define mode!");
            ros::Duration(0.5).sleep();
        }
        // q0 = current_state.pose.orientation.x;
        // q1 = current_state.pose.orientation.y;
        // q2 = current_state.pose.orientation.z;
        // q3 = current_state.pose.orientation.w;
        // pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
        // roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
        // yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;
        tf::quaternionMsgToTF(current_state.pose.orientation, quad);
        tf::Matrix3x3(quad).getRPY(roll, pitch, yaw);
        if (init)
        {
            yaw_value = yaw;
            init = false;
        }
        if (detect_error.flag == 1 && abs(detect_error.ex) < 120)
        {
            //"X Velocity [m/s] 向前
            if (sqrt(detect_error.ex * detect_error.ex + detect_error.ey * detect_error.ey) < 10)
            {
                desire_cmd_value[0] = 0;
            }
            else
            {
                desire_cmd_value[0] = 0.003 * (250 - detect_error.w);
            }
            // desire_cmd_value[0] = 0;

            //"Y Velocity [m/s] 向左
            desire_cmd_value[1] = 0.003 * detect_error.ex; // ex = 320-cx
            //"Z Velocity [m/s] 向上
            desire_cmd_value[2] = 0.003 * (detect_error.ey); // ey = 240-cy
            //if (abs(current_state.pose.position.z) - minz > 3.5)
            //{
            //   desire_cmd_value[2] = -0.2;
            //}
                
            fail_count = 0; 
            last_pos = detect_error.ex / abs(detect_error.ex);
            /*
            desire_cmd_value1[0] = desire_cmd_value1[0] + pid_w.pid_control(250, detect_error.w);//pid_control(desire, actual)
            desire_cmd_value1[1] = desire_cmd_value1[1] + pid_ey.pid_control(0, detect_error.ey);//pid_control(desire, actual)
            desire_cmd_value1[2] = desire_cmd_value1[2] + pid_ex.pid_control(0, detect_error.ex);//pid_control(desire, actual)

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

            ROS_INFO("Find Balloon!");

            cmd.desire_cmd[0] = desire_cmd_value[0];
            cmd.desire_cmd[1] = desire_cmd_value[1];
            cmd.desire_cmd[2] = desire_cmd_value[2];
            cmd.yaw_cmd = yaw_value;
        }
        else
        {
            if (detect_error.flag == 0)
            {
                fail_count += 1;
                if (fail_count > 5)
                {
                    desire_cmd_value[0] = 0;
                    desire_cmd_value[1] = 0;
                    desire_cmd_value[2] = 0.05;

                    cmd.desire_cmd[0] = desire_cmd_value[0];
                    cmd.desire_cmd[1] = desire_cmd_value[1];
                    cmd.desire_cmd[2] = desire_cmd_value[2];

                    ROS_WARN("Not Detect Balloon!");
                    // yaw_value += PI/20;
                    if (last_pos > 0) // ex > 0 框在左边，飞机在右边
                    {
                        yaw_value += PI / 60;
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
                    cmd.yaw_cmd = yaw_value;
                }
            }
            else if (detect_error.flag == 1 && abs(detect_error.ex) > 120)
            {
                desire_cmd_value[0] = 0;
                desire_cmd_value[1] = 0;
                desire_cmd_value[2] = 0;

                cmd.desire_cmd[0] = desire_cmd_value[0];
                cmd.desire_cmd[1] = desire_cmd_value[1];
                cmd.desire_cmd[2] = desire_cmd_value[2];

                ROS_WARN("Balloon Lost!");
                if (detect_error.ex > 0) // 框在左边，飞机在右边
                {
                    yaw_value += PI / 60;
                }
                else
                {
                    yaw_value -= PI / 60;
                }
                cmd.yaw_cmd = yaw_value;
            }
        } // end else
        t = t + 0.1;
        cmd.ext_time = t;
        cmd_pub.publish(cmd);
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
}

vector<Point> GetPoint(vector<Point> contour)
{
    cv::Rect rect = cv::boundingRect(cv::Mat(contour));
    Point up, down, left, right;
    up.x = rect.x + rect.width / 2;
    up.y = rect.y;
    vector<Point> p1;
    p1.push_back(up);
    down.x = rect.x + rect.width / 2;
    down.y = rect.y + rect.height;
    p1.push_back(down);
    left.x = rect.x;
    left.y = rect.y + rect.height / 2;
    p1.push_back(left);
    right.x = rect.x + rect.width;
    right.y = rect.y + rect.height / 2;
    p1.push_back(right);
    return p1;
}

int circle_detect(vector<Point> contour, float cx, float cy)
{

    int num = 0;
    float sum = 0, tmp = 0;
    int circle_x, circle_y;
    for (int i = 0; i < contour.size(); i++)
    {
        circle_x = contour[i].x;
        circle_y = contour[i].y;
        sum += sqrt((cx - circle_x) * (cx - circle_x) + (cy - circle_y) * (cy - circle_y));
        num += 1;
    }

    float mean = sum / num;
    sum = 0.0f;
    for (int i = 0; i < contour.size(); i++)
    {
        circle_x = contour[i].x;
        circle_y = contour[i].y;
        tmp = (sqrt((cx - circle_x) * (cx - circle_x) + (cy - circle_y) * (cy - circle_y)) - mean);
        sum += tmp * tmp;
    }
    return sum / num;
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
    double Km = (Bm + Gm + Rm) / 3;

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
    cv::Mat imgOriginal;
    Mat imgBalance;
    Mat imgHSV;
    Mat imgRGB;
    Mat imgGray;
    Mat output, binary;
    Mat element1, element2;
    vector<vector<Point>> contours;
    vector<vector<Point>> hulls;
    vector<Vec4i> hierachy;
    vector<Point> hull;
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
        cvtColor(imgRGB, imgHSV, COLOR_BGR2HSV); // Convert the captured frame from BGR to HSV
        // 因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
        for (int i = 0; i < imgHSV.rows; i++)
        {
            for (int j = 0; j < imgHSV.cols; j++)
            {

                int H = imgHSV.at<Vec3b>(i, j)[0];
                int S = imgHSV.at<Vec3b>(i, j)[1];
                int V = imgHSV.at<Vec3b>(i, j)[2];
                /*
                    opencv 的H范围是0~180，红色的H范围大概是(0~8)∪(160,180)
                    S是饱和度，一般是大于一个值,S过低就是灰色（参考值S>80)，
                    V是亮度，过低就是黑色，过高就是白色(参考值220>V>50)。
                */
                // if (( ((H > 0) && (H < 10) || (H > 150) && (H < 180)) && ((S >= 200 && S <= 255) || (S >= 30 && S <= 60))  && (V > 200 && V < 255) ))
                if ((((H >= 0) && (H <= 20) || (H >= 150) && (H <= 180)) && (S >= 43 && S <= 256) && (V > 120 && V < 256)))
                {
                    imgRGB.at<Vec3b>(i, j)[0] = 0;
                    imgRGB.at<Vec3b>(i, j)[1] = 0;
                    imgRGB.at<Vec3b>(i, j)[2] = 255;
                }
                else
                {
                    imgRGB.at<Vec3b>(i, j)[0] = 255;
                    imgRGB.at<Vec3b>(i, j)[1] = 255;
                    imgRGB.at<Vec3b>(i, j)[2] = 255;
                }
            }
        }
        cvtColor(imgRGB, imgGray, COLOR_BGR2GRAY);
        element1 = getStructuringElement(MORPH_RECT, Size(10, 10));
        element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
        
        dilate(imgGray, output, element1);
        erode(output, output, element2);
        imshow("rgb", imgRGB);
        imshow("gray", output); // show the gray image
        threshold(output, binary, 120, 255, THRESH_BINARY_INV);
        imshow("binary", binary); // show the thresholded image

        
        findContours(binary, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        Mat contour_img(output.size(), CV_8U, Scalar(0));
        drawContours(contour_img, contours, -1, Scalar(255), 3);

        float areas[100] = {0}, _max = 0;
        int _maxid = 0;

        if (!contours.empty() && !hierachy.empty()) // 如果有检测结果， 取最大值
        {
            int k = 0;
            std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
            std::vector<std::vector<cv::Point>>::const_iterator hull_itc = hulls.begin();

            while (itc != contours.end())
            {
                convexHull(*itc, hull);
                areas[k] = contourArea(hull);

                if (areas[k] > _max && areas[k] > 30 && areas[k] < 60000)
                {
                    _max = areas[k];
                    _maxid = k;
                }
                *itc++;
                k++;
                *hull_itc++;
            }

            // for(int _i = 0;_i < 10;_i++){cout << "areas " << _i << " = " << areas[_i] << endl;}
            // cout << "最大面积" << _max << " " << _maxid << endl;

            drawContours(contour_img, contours, -1, Scalar(255), 1);
            // drawContours(contour_img, hulls, -1, Scalar(255), 1);

            itc = contours.begin();
            for (int _i = 0; _i < _maxid; _i++)
            {
                *itc++;
            }
            Rect rect = boundingRect(*itc);
            float var = circle_detect(*itc, rect.x + rect.width / 2, rect.y + rect.height / 2);
            // cout << "方差： " << var << endl; // x,y,w,h
            // Rect rect = boundingRect(hulls[_maxid]);

            if (_max > 30 && _max < 60000 && var < 100)
            {
                rectangle(imgOriginal, rect, Scalar(255, 0, 0));
                ROS_INFO("Detect Result: [%d, %d, %d, %d]", rect.x, rect.y, rect.width, rect.height);
                detect_error.ex = imgOriginal.cols / 2 - (rect.x + rect.width / 2);
                detect_error.ey = imgOriginal.rows / 2 - (rect.y + rect.height / 2);
                detect_error.w = rect.width;
                detect_error.flag = 1;
            }
            else // 检测到轮廓，但是不符合气球约束
            {
                detect_error.ex = 0;
                detect_error.ey = 0;
                detect_error.w = 250;
                detect_error.flag = 0;
            }

        } // end if
        else
        { // 没有检测结果
            detect_error.ex = 0;
            detect_error.ey = 0;
            detect_error.w = 250;
            detect_error.flag = 0;
        }
        imshow("raw_img", imgOriginal);
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }
} // end main func

void image_raw_sub(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::Image current_state = *msg;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(cv_ptr->image, image_from_topic, COLOR_RGB2BGR);
}

void state_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_state = *msg;
}

void ext_on_cb(const std_msgs::Bool::ConstPtr &msg)
{
    ext_on = msg->data;
}
