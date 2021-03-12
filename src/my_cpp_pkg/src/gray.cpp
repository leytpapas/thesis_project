#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <sys/time.h>
#include <stdio.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>

// #include <QPULib.h>
// #include "geometry_msgs/Bool.h"
#include <sstream>
// #include <cstdio.h>
#include <cmath>
#define RASPI //in order to not do unecassary stuff
#define FASTPOW
#define RGB
#define HSV
// #define CUSTOM
// #define GRAYWORLD
// #define LUM
#define OMP
#define MYSQRT  //in order to use a quick sqrt with decimal precision instead of the built-in 'sqrt'

// #define DEBUG
// #define DEBUG_PC

using namespace cv;
using namespace std;

int line_width = 200;
int seq = 0;
int row = -1;
int col = -1;

double tD = 20; //optimal for saturation filtering
int tU = 95;    //optimal for saturation filtering
// double tD = 50;
// double tU = 170; //optimal??
uchar euclidean_distance = 40;
int lumU = 40;
int lumD = 150;
uchar pix_off = 50; // trial and error
double eps = 0.1;
Point last_turn = Point(0, 0); // x=y=0 == NULL for us
Point point_to_go;
vector<uchar> euclidean_color{0, 0, 255}; //Blue (RGB values)
cv_bridge::CvImageConstPtr cv_ptr;
sensor_msgs::Image img_msg; // >> message to be sent
ros::Subscriber sub;
ros::Subscriber sub2;
cv_bridge::CvImage cv_ptr_out;
ros::Publisher image_pub;

#ifdef DEBUG_PC
cv_bridge::CvImage cv_ptr_out_rgb;
ros::Publisher image_pub_rgb;
#endif

uint16_t *lookupTable;

ros::NodeHandle *n;
#ifndef SQRT
void generate(uchar input);
#endif
void image_data(const sensor_msgs::ImageConstPtr &msg);

// #define min_f(a, b, c) (fminf(a, fminf(b, c)))
// #define max_f(a, b, c) (fmaxf(a, fmaxf(b, c)))
#define min(a, b, c) (min(a, min(b, c)))
#define max(a, b, c) (max(a, max(b, c)))

float my_sqrt(float x)
{
    unsigned int i = *(unsigned int *)&x;
    // adjust bias
    i += 127 << 23;
    // approximation of square root
    i >>= 1;
    return *(float *)&i;
}

double fastPow(double a, double b)
{
    union
    {
        double d;
        int x[2];
    } u = {a};
    u.x[1] = (int)(b * (u.x[1] - 1072632447) + 1072632447);
    u.x[0] = 0;
    return u.d;
}

int calcDistEuclidean(Point a, Point b, bool complex = false)
{
    return sqrt(fastPow(a.x - b.x, 2) + fastPow(a.y - b.y, 2));
}

static bool cmp(const vector<Point> &lhs, const vector<Point> &rhs)
{
    return contourArea(lhs) > contourArea(rhs);
}

Point centroid(Point *a, Point *b)
{
    return Point(a->x + b->x / 2, a->y + b->y);
}

Point centroid(vector<Point> list_points)
{
    int sumx = 0;
    int sumy = 0;
    for (int i = 0; i < list_points.size(); i++)
    {
        sumx += list_points[i].x;
        sumy += list_points[i].y;
    }
    return Point(sumx / (list_points.size() > 0 ? list_points.size() : 1), sumy / (list_points.size() > 0 ? list_points.size() : 1));
}

void add_group_by_distance(vector<Point> *src, Point point)
{
    for (int i = 0; i < (*src).size(); i++)
    {
        if (hypot((*src)[i].x - point.x, (*src)[i].y - point.y) < line_width)
        {
            (*src)[i] = centroid(&(*src)[i], &point);
            return;
        }
    }
    (*src).insert((*src).end(), point);
}

vector<int> calcuHisto(const IplImage *src_pic, int anzBin)
{
    CvSize size = cvGetSize(src_pic);
    double binSize = 256.0 / anzBin;  //new definition
    vector<int> histogram(anzBin, 0); //i don't know if this works so I
                                      //so I will leave it

    //goes through all rows
    for (int y = 0; y < size.height; y++)
    {
        //grabs an entire row of the imageData
        const uchar *src_pic_point = (uchar *)(src_pic->imageData + y * src_pic->widthStep);

        //goes through each column
        for (int x = 0; x < size.width; x++)
        {
            //for each bin
            for (int z = 0; z < anzBin; z++)
            {
                //check both upper and lower limits
                if (src_pic_point[x] >= z * binSize && src_pic_point[x] < (z + 1) * binSize)
                {
                    //increment the index that contains the point
                    histogram[z]++;
                }
            }
        }
    }
    return histogram;
}

// A binary search based function
// to find the position
// where item should be on the lookup table
// in a[low..high]
uchar binarySearch(uint16_t a[], uint16_t item,
                   uchar low, uchar high)
{
    if (high <= low)
        return (item > a[low]) ? (low + 1) : low;

    uchar mid = (low + high) / 2;

    if (item == a[mid])
        return mid + 1;

    if (item > a[mid])
        return binarySearch(a, item,
                            mid + 1, high);
    return binarySearch(a, item, low,
                        mid - 1);
}

// Returns true if x is in range [low..high], else false
bool inRange(int low, int high, int x)
{
    return ((x - high) * (x - low) <= 0);
}

Point filterWorld(Mat *src, Mat *dst)
{

#ifdef GRAYWORLD
    double R = 0;
    double B = 0;
    double G = 0;
#endif
#ifdef DEBUG_PC
    Mat *dst2 = (Mat *)&cv_ptr_out_rgb.image;
#endif
    int row = src->rows;
    int col = src->cols;

    //Create a Mat object of the same size as the input image
    // Mat dst(row, col, CV_8UC1);
    if (euclidean_distance < 1)
    {
        ROS_INFO("Euclidean distance should be grated than 1 but was given %d", euclidean_distance);
        return Point(0, 0);
    }
    //Get the RGB value of each pixel of the input image
    uchar r, g, b;
    // medianBlur(*src, *src, 3);
    // normalize(*src,*src,tD,tU,NORM_MINMAX); // this or Gray world help very much, but are not time efficient
    medianBlur(*src, *src, 3);
    // GaussianBlur(*src, *src, Size(3, 3), 0);
    // gaussian should be better but is slower

#ifdef GRAYWORLD
#ifdef OMP
#pragma omp parallel for reduction(+ \
                                   : R, G, B) shared(src) num_threads(4)
#endif
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            R += 1.0 * src->ptr<uchar>(i, j)[0];
            G += 1.0 * src->ptr<uchar>(i, j)[1];
            B += 1.0 * src->ptr<uchar>(i, j)[2];
        }
    }
#ifdef OMP
#pragma omp ordered num_threads(4)
#endif
    //Average
    R /= ((row / 2) * col);
    B /= ((row / 2) * col);
    G /= ((row / 2) * col);
#ifdef DEBUG
    printf("Average B, G, R:%.5f %.5f %.5f\n", B, G, R);
#endif
    //Calculate the average value of the three RGB channels of the image
    double GrayValue = (R + G + B) / 3;
#ifdef DEBUG
    printf("GrayValue %.5f\n", GrayValue);
#endif
    //Calculate the gain coefficient of the three channels
    double kr = GrayValue / R;
    double kg = GrayValue / G;
    double kb = GrayValue / B;
#ifdef DEBUG
    printf("kb, kg, kr: %.5f %.5f %.5f\n", kb, kg, kr);
#endif
#endif
    //-- According to the diagonal model of Von Kries. For each pixel in the image C adjustment adjusts its RGB components
    //-- After that filter using euclidean distance;
    int i, j, temp;
    // uint16_t r_p, b_p, g_p;
    double hsv_min, hsv_max;
    double h, s, v;
    uchar delta;
    int tid;
#ifdef OMP
#pragma omp parallel private(i, j, tid, temp, hsv_min, hsv_max, h, s, v, r, g, b) shared(row, col, src, dst, euclidean_distance, euclidean_color) num_threads(4) 
    {
#pragma omp for collapse(2)
#endif        
        printf("%d :number of threads = %d\n", omp_get_thread_num(), omp_get_num_threads());

        for (i = 0; i < row; i++)
        {
            // #pragma omp for private(pixel)// schedule(dynamic)
            for (j = 0; j < col; j++)
            {
                r = src->ptr<uchar>(i, j)[0]; //*p++; //src r
                g = src->ptr<uchar>(i, j)[1]; //*p++; //src g
                b = src->ptr<uchar>(i, j)[2]; //*p++; //src b
#ifdef GRAYWORLD
                r = (int)(kr * r) > 255 ? 255 : (int)(kr * r);
                g = (int)(kg * g) > 255 ? 255 : (int)(kg * g);
                b = (int)(kb * b) > 255 ? 255 : (int)(kb * b);
#endif
#ifdef DEBUG_PC
                dst2->ptr<uchar>(i, j)[0] = r;
                dst2->ptr<uchar>(i, j)[1] = g;
                dst2->ptr<uchar>(i, j)[2] = b;
#endif
#ifdef HSV
                // MinMax() Needs at least 4 comparisons
                hsv_min = min(r, g, b);
                hsv_max = max(r, g, b);
                v = hsv_max;
                delta = hsv_max - hsv_min;
                if (hsv_max == 0 || delta == 0)
                {
                    s = 0;
                    h = 0;
                }
                else
                {
                    s = (delta / 255) / hsv_max;
                    if (r == hsv_max)                   // > is bogus, just keeps compilor happy
                        h = 60 * ((g - b) / delta) + 0; // between yellow & magenta
                    else if (g == hsv_max)
                        h = 60 * ((b - r) / (delta)) + 120; // between cyan & yellow
                    else
                        h = 60 * ((r - g) / (delta)) + 240; // between magenta & cyan
                    if (h < 0.0)
                        h += 360.0;
                }
#endif
#if defined(RGB) && defined(MYSQRT) && defined(LUM)
                // sqrt( 0.299*R^2 + 0.587*G^2 + 0.114*B^2 ) == Luminance of pixel
                temp = (uchar)(my_sqrt(0.299 * fastPow(r - euclidean_color.at(0), 2) + 0.587 * fastPow(g - euclidean_color.at(1), 2) + 0.144 * fastPow(b - euclidean_color.at(2), 2)));
#elif defined(RGB) && defined(MYSQRT)
            temp = (uchar)(my_sqrt(fastPow(r - euclidean_color.at(0), 2) + fastPow(g - euclidean_color.at(1), 2) + fastPow(b - euclidean_color.at(2), 2)));
#elif defined(RGB)
            temp = (uchar)(sqrt(fastPow(r - euclidean_color.at(0), 2) + fastPow(g - euclidean_color.at(1), 2) + fastPow(b - euclidean_color.at(2), 2)));
#endif

#if defined(RGB) and defined(HSV) and defined(CUSTOM)
                if ((r < lumD && g < lumD && b < lumD) || (r > lumU && g > lumU && b > lumU))
                {
                    dst->ptr<uchar>(i, j)[0] = 0;
                }
                else if (!inRange(0, 100, h / 2) && temp < euclidean_distance)
                {
                    dst->ptr<uchar>(i, j)[0] = 255; // one channel
                }
#elif defined(RGB) and defined(HSV)
            if (!inRange(0, 100, h / 2) && temp < euclidean_distance)
            {
                dst->ptr<uchar>(i, j)[0] = 255; // one channel
            }
#elif defined(HSV) and defined(CUSTOM)
            if ((r < lumD && g < lumD && b < lumD) || (r > lumU && g > lumU && b > lumU))
            {
                dst->ptr<uchar>(i, j)[0] = 0;
            }
            else if (!inRange(0, 100, h / 2))
            {
                dst->ptr<uchar>(i, j)[0] = 255; // one channel
            }
#elif defined(HSV)

            if (!inRange(0, 100, h / 2))
            {
                dst->ptr<uchar>(i, j)[0] = 255; // one channel
            }
#elif defined(RGB) and defined(CUSTOM)
            if ((r < lumD && g < lumD && b < lumD) || (r > lumU && g > lumU && b > lumU))
            {
                dst->ptr<uchar>(i, j)[0] = 0;
            }
            else if (temp < euclidean_distance)
            {
                dst->ptr<uchar>(i, j)[0] = 255; // one channel
            }
#elif defined(RGB)
            if (temp < euclidean_distance)
            {
                dst->ptr<uchar>(i, j)[0] = 255; // one channel
            }
#endif
                else
                {
                    dst->ptr<uchar>(i, j)[0] = 0;
                }
            }
        }
#ifdef OMP
    }
#endif
    morphologyEx(*dst, *dst, MORPH_OPEN, getStructuringElement(MORPH_CROSS, Size(3, 3)));
    morphologyEx(*dst, *dst, MORPH_CLOSE, getStructuringElement(MORPH_CROSS, Size(3, 3)));
    vector<vector<Point>> contours;
    findContours(*dst, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.size() == 0)
    {
#ifdef DEBUG
        cout << "0 contours" << endl;
#endif

        return Point(0, 0);
    }
    std::sort(contours.begin(), contours.end(), cmp); // 12 32 45 71(26 33 53 80)

    // vector<Point> max_contour = *max_element(contours.begin(), contours.end(), cmp);
    if (contourArea(contours[0]) < tU) // this is for full resolution
    {
#ifdef DEBUG
        cout << "Didnt find any" << endl;
#endif
        return Point(0, 0);
    }
#ifdef DEBUG_PC
    drawContours(*dst2, contours, 0, Scalar(255, 0, 0), 2, LINE_8, 0, 0);
#endif

    vector<vector<Point>> list_exits(5); //list_exits = {"South": [], "North": [], "West": [], "East": [], "rest of the points":[]}

    double epsilon = 0.01 * arcLength(contours[0], false); //eps_line
    vector<Point> aprox;
    approxPolyDP(contours[0], aprox, epsilon, true);
    for (i = 0; i < aprox.size(); i++)
    {
        Point point = aprox[i];
        if (point.y > row - pix_off) // point on south border
            add_group_by_distance(&list_exits[0], point);
        else if (point.y < pix_off) // point on north borderGrou
            add_group_by_distance(&list_exits[1], point);
        else if (point.x < pix_off) // point on west border
            add_group_by_distance(&list_exits[2], point);
        else if (point.x > col - pix_off) // point on east border
            add_group_by_distance(&list_exits[3], point);
        else
            list_exits[4].insert(list_exits[4].end(), point);
    }
    int num_exits = 0;
    for (i = 0; i < list_exits.size() - 1; i++) // Only count South,North,East,West
    {
        num_exits += list_exits[i].size();
    }
#ifdef DEBUG
    cout << "Num exits " << num_exits << endl;
#endif
    point_to_go.x = 0;
    point_to_go.y = 0;
    if (num_exits > 2 and (last_turn.x == 0 && last_turn.y == 0))
    {
#ifdef DEBUG
        cout << "choose the closest to the middle of the screen" << endl;
#endif
        //choose the min,with euclidean distance
        for (i = 0; i < list_exits.size(); i++)
        {
            for (j = 0; j < list_exits[i].size(); j++)
            {
                Point(col / 2, row);
                if ((point_to_go.x == 0 && point_to_go.y == 0) or calcDistEuclidean(Point(col / 2, row / 2), list_exits[i][j]) < calcDistEuclidean(Point(col / 2, row / 2), point_to_go))
                {
                    point_to_go = list_exits[i][j];
                }
            }
        }
        last_turn = point_to_go;
    }
    else if (last_turn.x != 0 || last_turn.y != 0)
    {
#ifdef DEBUG
        cout << "choose the closest to the previous chosen point (for one last time)" << endl;
#endif
        //choose the min,with euclidean distance
        for (i = 0; i < list_exits.size(); i++)
        {
            for (j = 0; j < list_exits[i].size(); j++)
            {
                if ((point_to_go.x == 0 && point_to_go.y == 0) or calcDistEuclidean(last_turn, list_exits[i][j]) < calcDistEuclidean(last_turn, point_to_go))
                {
                    point_to_go = list_exits[i][j];
                }
            }
        }
        last_turn.x = 0;
        last_turn.y = 0;
    }
    else if (list_exits[4].size() > 0)
    {
#ifdef DEBUG
        cout << "choose the intersection" << endl;
#endif
        point_to_go = list_exits[4][0];
        last_turn = point_to_go;
    }
    else
    {
#ifdef DEBUG
        cout << "choose the highest seen" << endl;
#endif
        for (i = 0; i < list_exits.size(); i++)
        {
            for (j = 0; j < list_exits[i].size(); j++)
            {
                if (point_to_go.y < list_exits[i][j].y)
                {
                    point_to_go = list_exits[i][j];
                }
            }
        }
    }

    return point_to_go;
}

Mat GetSquareImage(const Mat &img, int target_width = 500)
{
    int width = img.cols,
        height = img.rows;

    Mat square = Mat::zeros(target_width, target_width, img.type());

    int max_dim = (width >= height) ? width : height;
    float scale = ((float)target_width) / max_dim;
    Rect roi;
    if (width >= height)
    {
        roi.width = target_width;
        roi.x = 0;
        roi.height = height * scale;
        roi.y = (target_width - roi.height) / 2;
    }
    else
    {
        roi.y = 0;
        roi.height = target_width;
        roi.width = width * scale;
        roi.x = (target_width - roi.width) / 2;
    }

    resize(img, square(roi), roi.size());

    return square;
}

void camera_info(const sensor_msgs::ImageConstPtr &msg)
{
    row = msg->height;
    col = msg->width;
    pix_off = msg->height / 10;

    cv_ptr_out = [] {
        cv_bridge::CvImage ret;
        std_msgs::Header temp;
        ret.header = temp;
        ret.header.frame_id = "my_cpp_node";
        ret.header.seq = 0;
        ret.header.stamp = ros::Time::now();
        ret.encoding = sensor_msgs::image_encodings::MONO8;
        ret.image = Mat(row, col, CV_8UC1);
        return ret;
    }();
    ROS_INFO("Camera input frame: %d*%d", col, row);
#ifdef DEBUG_PC

    cv_ptr_out_rgb = [] {
        cv_bridge::CvImage ret;
        std_msgs::Header temp;
        ret.header = temp;
        ret.header.frame_id = "my_cpp_node";
        ret.header.seq = 0;
        ret.header.stamp = ros::Time::now();
        ret.encoding = sensor_msgs::image_encodings::RGB8;
        ret.image = Mat(row, col, CV_8UC3);
        return ret;
    }();
    ROS_INFO("Camera input frame: %d*%d", col, row);
#endif
#ifndef SQRT
    generate(255);
#endif
    // sleep(1);
    sub2 = n->subscribe("/usb_cam_node/image_raw", 1, image_data);
    image_pub = n->advertise<sensor_msgs::Image>("mybot_out/im_out", 1);

#ifdef DEBUG_PC
    image_pub_rgb = n->advertise<sensor_msgs::Image>("mybot_out/im_out_rgb", 1);
#endif
    sub.shutdown(); //erase this function's subscribtion
}

void image_data(const sensor_msgs::ImageConstPtr &msg)
{
    struct timeval beg, end;
    double elapsed_time;
    Mat result;
    int param;
    int *eucl_c;

    if (n->getParam("euclidean_dist", param) && param != euclidean_distance)
    {
        euclidean_distance = param;
#ifndef SQRT
        generate(euclidean_distance);
#endif
    }
    if (n->getParam("euclidean_color", *eucl_c))
    {
        euclidean_color.at(0) = (uchar)eucl_c[0];
        euclidean_color.at(1) = (uchar)eucl_c[1];
        euclidean_color.at(2) = (uchar)eucl_c[2];
        // euclidean_color = eucl_c;
    }
    if (n->getParam("tU", param) && param != tU)
    {
        tU = param;
    }
    if (n->getParam("tD", param) && param != tD)
    {
        tD = param;
    }
    if (n->getParam("lumU", param) && param != lumU)
    {
        lumU = param;
    }
    if (n->getParam("lumD", param) && param != lumD)
    {
        lumD = param;
    }
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8); // modify shared msg,be careful with other callbacks of same topic
        gettimeofday(&beg, NULL);
        cout << filterWorld((Mat *)&cv_ptr->image, (Mat *)&cv_ptr_out.image) << endl;
        gettimeofday(&end, NULL);
#ifndef RASPI
        imshow("Received", cv_ptr->image);
#endif
        // compute and print the elapsed time in millisec
        elapsed_time = (end.tv_sec - beg.tv_sec) * 1000.0;    // sec to ms
        elapsed_time += (end.tv_usec - beg.tv_usec) / 1000.0; // us to ms

        printf("Time/frame: %f ms, %f fps: \n", elapsed_time, 1000 / elapsed_time);
        cv_ptr_out.header.seq = seq++;
        cv_ptr_out.header.stamp = ros::Time::now();
        image_pub.publish(cv_ptr_out.toImageMsg());
#ifdef DEBUG_PC
        cv_ptr_out_rgb.header.seq = seq++;
        cv_ptr_out_rgb.header.stamp = ros::Time::now();
        image_pub_rgb.publish(cv_ptr_out_rgb.toImageMsg());
#endif
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// Generate lookuptable for sqrt values of single
// precision for comparison with euclidean/lum
// till 255 with 154 being the max(euclidean/lum)
void generate(uchar input)
{
    if (input > 255 || 0 > input)
        throw "Correct Input: <255 and >0";
    delete lookupTable;
    lookupTable = new uint16_t[input];
    for (int i = 0; i < input; i++)
    {
        lookupTable[i] = pow(i, 2);
    }
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle no;
    n = &no;
#ifndef RASPI
    namedWindow("Received", WINDOW_AUTOSIZE);
#endif
    sub = no.subscribe("/usb_cam_node/image_raw", 1, camera_info);
    ROS_INFO("Waiting for image_raw... (at least one publish)");
    // ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/usb_cam_node/image_raw");
    ros::spin();

    return 0;
}
