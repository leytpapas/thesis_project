#include <iostream>
#include <numeric>
#include <bitset>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <sys/time.h>
#include <stdio.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "mybot_msg/navigate_msg.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>

#include <sstream>
// #include <cstdio.h>
#include <cmath>
// #define MYSQRT // Use our sqrt instead of the built-in std::sqrt()
// #define FASTPOW // use fast_pow function instead of the built-in std::pow()
#define INLINE_POWER // use x*x instead of pow(x,2)
#define RGB
#define HSV
#define CUSTOM // eliminate pixels with too low/too high values
#define GRAYWORLD
// #define LUM // Adds luminocity values in rgb euclidean calculation
// #define OMP
#define NUM_THR 4
// #define COLOR_DECREASE 10 //8bit --> 256/8 = 32 => 5 bit
#define DEBUG
// #define DEBUG_PC
#define LOOKUP

using namespace cv;
using namespace std;

int line_width = 400;
int seq = 0;
int row = -1;
int col = -1;

// double tD = 20; //optimal for saturation filtering
int contour_area_min = 95; //optimal for saturation filtering

// double tD = 50;
// double tU = 170; //optimal??
uint8_t euclidean_distance = 40;
int luminocity_up = 40;
int luminocity_low = 150;
uint8_t pix_off = 50; // trial and error
double eps = 0.1;
Point last_turn = Point(0, 0); // x=y=0 == NULL for us
Point point_to_go;

static Point robot_view_point(col / 2, row);

vector<uint8_t> euclidean_color{0, 0, 255}; //Blue (RGB values)
cv_bridge::CvImageConstPtr cv_ptr;
sensor_msgs::Image img_msg; // >> message to be sent
ros::Subscriber sub;
ros::Subscriber sub2;
cv_bridge::CvImage cv_ptr_out;
ros::Publisher image_pub;
ros::Publisher chatter_pub;

vector<double> fps_counter(2, 0);

#if defined(DEBUG_PC)
Mat *dst2;
cv_bridge::CvImage cv_ptr_out_rgb;
ros::Publisher image_pub_rgb;
#endif

ros::NodeHandle *n;

#if defined(LOOKUP)
// static uchar lookupTable[26425];

static uchar lookupTable_new[26 * 26 * 26];
// static bool bool_lookupTable[26425];
// static bool bool_lookupTable2[256 / COLOR_DECREASE][256 / COLOR_DECREASE][256 / COLOR_DECREASE];
// static uchar lookupTable2[256 / COLOR_DECREASE][256 / COLOR_DECREASE][256 / COLOR_DECREASE];
bool params_changed = true;

void generate();
#endif
void image_data(const sensor_msgs::ImageConstPtr &msg);

// #define min_f(a, b, c) (fminf(a, fminf(b, c)))
// #define max_f(a, b, c) (fmaxf(a, fmaxf(b, c)))

// #define min_3(a, b, c) (min(a, min(b, c)))
// #define max_3(a, b, c) (max(a, max(b, c)))

uchar __inline__ index_formula(uchar r, uchar g, uchar b)
{
    return (r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10;
}

float __inline__ min_3(float a, float b, float c)
{
    return min(a, min(b, c));
}

float __inline__ max_3(float a, float b, float c)
{
    return max(a, max(b, c));
}

#if defined(SQRT1)
// https://bits.stephan-brumme.com/squareRoot.html
//workd only for 32 bits, which suits us
float __inline__ custom_sqrt(float x)
{
    static unsigned int i = *(unsigned int *)&x;
    // adjust bias
    i += 127 << 23;
    // approximation of square root
    i >>= 1;
    return *(float *)&i;
}
#else

float __inline__ custom_sqrt(float x)
{
    static union
    {
        int i;
        float x;
    } u;

    u.x = x;
    u.i = (1 << 29) + (u.i >> 1) - (1 << 22);
    return u.x;
}

#endif

double fastPow(double a, double b)
{
    static union
    {
        double d;
        int x[2];
    } u = {a};
    u.x[1] = (int)(b * (u.x[1] - 1072632447) + 1072632447);
    u.x[0] = 0;
    return u.d;
}

unsigned short __inline__ fast_power_two(uint8_t a)
{
    return a * a;
}

static int __inline__ calcDistEuclidean(Point a, Point b)
{
#ifdef MYSQRT

#if defined(INLINE_POWER)
    return custom_sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
#else
    return custom_sqrt(fastPow(a.x - b.x, 2) + fastPow(a.y - b.y, 2));
#endif
#else
#if defined(INLINE_POWER)
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
#else
    return sqrt(fastPow(a.x - b.x, 2) + fastPow(a.y - b.y, 2));
#endif
#endif
}

static bool __inline__ cmp(const vector<Point> &lhs, const vector<Point> &rhs)
{
    return contourArea(lhs) > contourArea(rhs);
}

static __inline__ void sort_custom(uint8_t *d)
{
    int i, j;
    for (i = 1; i < 9; i++)
    {
        uint8_t tmp = d[i];
        for (j = i; j >= 1 && tmp < d[j - 1]; j--)
            d[j] = d[j - 1];
        d[j] = tmp;
    }
}

Point __inline__ centroid(Point *a, Point *b)
{
    return Point((a->x + b->x) / 2, (a->y + b->y) / 2);
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

void add_in_group_by_distance(vector<Point> *src, Point point)
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

double __inline__ angle_between_points(Point a, Point b)
{
    float angle = atan2(a.y - b.y, a.x - b.x) * (180 / M_PI);
    if (angle < 0)
    {
        angle += 180;
    }
    return angle;
}

// Returns true if x is in range [low..high], else false
bool __inline__ inRange(int low, int high, int x)
{
    return ((x - high) * (x - low) <= 0);
}

static __inline__ void normal_method(Mat *src, Mat *dst)
{
    //Get the RGB value of each pixel of the input image
    
#if defined(GRAYWORLD)
    static double R = 0;
    static double B = 0;
    static double G = 0;
#endif
    // normalize(*src,*src,tD,tU,NORM_MINMAX); // this or Gray world help very much, but are not time efficient
    medianBlur(*src, *src, 3);
    // GaussianBlur(*src, *src, Size(3, 3), 0);
    // gaussian should be better but is slower
#if defined(DEBUG_PC)
    dst2 = (Mat *)&cv_ptr_out_rgb.image;
#endif

#if defined(GRAYWORLD)
    static uchar *ptr;
#if defined(OMP)
    // omp_set_num_threads(NUM_THR);
#pragma omp parallel private(ptr) shared(R, G, B, src) num_threads(NUM_THR)
    {
// #pragma omp for collapse(1) reduction(+ \
//                                       : R, G, B)
// #pragma omp for schedule(static)

#endif
        for (unsigned i = 0; i < row; ++i)
        {
            ptr = src->ptr<uint8_t>(i); //get row
#if defined(OMP)
// #pragma omp for
#pragma omp for collapse(1) reduction(+ : R, G, B)
#endif
            for (unsigned j = 0; j < col; ++j)
            {
                R += ptr[0];
                B += ptr[1];
                G += ptr[2];
                ptr += 3;
            }
        }
#if defined(OMP)
    }
#endif
#endif
    // medianBlur(*src, *src, 3);

#if defined(OMP)
// omp_set_num_threads(NUM_THR);
#pragma omp ordered
#endif
#if defined(GRAYWORLD)
    //Average
    R /= ((row / 2) * col);
    B /= ((row / 2) * col);
    G /= ((row / 2) * col);
    //Calculate the average value of the three RGB channels of the image
    double GrayValue = (R + G + B) / 3;
    //Calculate the gain coefficient of the three channels
    double kr = GrayValue / R;
    double kg = GrayValue / G;
    double kb = GrayValue / B;
#if defined(DEBUG)
    printf("GrayValue %.5f\n", GrayValue);
    printf("Average B, G, R:%.5f %.5f %.5f\n", B, G, R);
    printf("kb, kg, kr: %.5f %.5f %.5f\n", kb, kg, kr);
#endif

#endif
    //-- According to the diagonal model of Von Kries. For each pixel in the image C adjustment adjusts its RGB components
    //-- After that filter using euclidean distance;
    static uint32_t i, j;
    // uint16_t r_p, b_p, g_p;
    static uint8_t r = 0; //*p++; //src r
    static uint8_t g = 0; //*p++; //src b
    static uint8_t b = 0;
    static uint8_t *src_ptr;
    static uint8_t *dst_ptr;
    static uint8_t *dst2_ptr;

#if defined(OMP)
    omp_set_num_threads(NUM_THR);
#if defined(GRAYWORLD)
#pragma omp parallel num_threads(NUM_THR) private(i, j, r, g, b, src_ptr, dst_ptr, dst2_ptr) shared(row, col, src, dst, kr, kg, kb, euclidean_distance, euclidean_color, lookupTable_new)
#else
#pragma omp parallel num_threads(NUM_THR) private(i, j, r, g, b, src_ptr, dst_ptr, dst2_ptr) shared(row, col, src, dst, euclidean_distance, euclidean_color, lookupTable_new)
#endif
    {

#endif

#pragma omp for schedule(static)
        for (i = 0; i < row; i++)
        {

            src_ptr = src->ptr<uint8_t>(i);
            dst_ptr = dst->ptr<uint8_t>(i);
#if defined(DEBUG_PC)
            dst2_ptr = dst2->ptr<uint8_t>(i);
#endif

#pragma omp parallel for collapse(1) //schedule(dynamic)
            for (j = 0; j < col; j++)
            {
                //get R G B values
                r = src_ptr[0];
                g = src_ptr[1];
                b = src_ptr[2];
                src_ptr += 3;

#if defined(GRAYWORLD)
                r = (uint8_t)((kr * r) > 255 ? 255 : kr * r);
                g = (uint8_t)((kg * g) > 255 ? 255 : kg * g);
                b = (uint8_t)((kb * b) > 255 ? 255 : kb * b);
#endif

#if defined(DEBUG_PC)
                dst2_ptr[0] = r;
                dst2_ptr[1] = g;
                dst2_ptr[2] = b;
                dst2_ptr += 3;
#endif
#if defined(LOOKUP)
                // *dst_ptr = lookupTable[((r / 10) << 10) + ((g / 10) << 5) + (b / 10)];
                *dst_ptr = lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10];
                // *dst_ptr = lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE];
#else

#if defined(HSV)
            double h, s, v;
            uint8_t hsv_min, hsv_max;
            uint8_t delta;

            // MinMax() Needs at least 4 comparisons
            hsv_min = min_3(r, g, b);
            hsv_max = max_3(r, g, b);
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
            unsigned int temp = custom_sqrt((0.299 * (r - euclidean_color.at(0)) * (r - euclidean_color.at(0))) + (0.587 * (g - euclidean_color.at(1)) * (g - euclidean_color.at(1))) + (0.144 * (b - euclidean_color.at(2)) * (b - euclidean_color.at(2))));
#elif defined(RGB) && defined(MYSQRT)
            unsigned int temp = custom_sqrt((r - euclidean_color.at(0)) * (r - euclidean_color.at(0)) + (g - euclidean_color.at(1)) * (g - euclidean_color.at(1)) + (b - euclidean_color.at(2)) * (b - euclidean_color.at(2)));
#elif defined(RGB)
            unsigned int temp = sqrt((r - euclidean_color.at(0)) * (r - euclidean_color.at(0)) + (g - euclidean_color.at(1)) * (g - euclidean_color.at(1)) + (b - euclidean_color.at(2)) * (b - euclidean_color.at(2)));
#endif
#if defined(RGB) and defined(HSV) and defined(CUSTOM)
            if ((r < luminocity_low && g < luminocity_low && b < luminocity_low) || (r > luminocity_up && g > luminocity_up && b > luminocity_up))
            {
                *dst_ptr = 0;
            }
            else if (!inRange(0, 100, h / 2) && temp < euclidean_distance)
            {
                *dst_ptr = 255;
            }
#elif defined(RGB) and defined(HSV)
            if (!inRange(0, 100, h / 2) && temp < euclidean_distance)
            {
                *dst_ptr = 255;
            }
#elif defined(HSV) and defined(CUSTOM)
            if ((r < luminocity_low && g < luminocity_low && b < luminocity_low) || (r > luminocity_up && g > luminocity_up && b > luminocity_up))
            {
                *dst_ptr = 0;
            }
            else if (!inRange(0, 100, h / 2))
            {
                *dst_ptr = 255;
            }
#elif defined(HSV)
            if (!inRange(0, 100, h / 2))
            {
                *dst_ptr = 255;
            }
#elif defined(RGB) and defined(CUSTOM)
            if ((r < luminocity_low && g < luminocity_low && b < luminocity_low) || (r > luminocity_up && g > luminocity_up && b > luminocity_up))
            {
                *dst_ptr = 0;
            }
            else if (temp < euclidean_distance)
            {
                *dst_ptr = 255;
            }
#elif defined(RGB)
            if (temp < euclidean_distance)
            {
                *dst_ptr = 255;
            }
#endif
            else
            {
                *dst_ptr = 0;
            }
#endif
                dst_ptr += 1;
            }
#if defined(OMP)
        }
#endif
    }
}

static __inline__ void continous_method(Mat *src, Mat *dst)
{
    //Get the RGB value of each pixel of the input image

#if defined(GRAYWORLD)
    double R = 0;
    double B = 0;
    double G = 0;
#endif
    medianBlur(*src, *src, 3);
    // GaussianBlur(*src, *src, Size(3, 3), 0);
    // gaussian should be better but is slower
#if defined(DEBUG_PC)
    dst2 = (Mat *)&cv_ptr_out_rgb.image;
#endif

#if defined(GRAYWORLD)
    static uchar *ptr;
    ptr = src->ptr<uint8_t>(0); //get row
#if defined(OMP)
    // omp_set_num_threads(NUM_THR);
#pragma omp parallel firstprivate(ptr) shared(R, G, B, src) num_threads(NUM_THR)
    {
#endif

#if defined(OMP)

#pragma omp for collapse(1) reduction(+ \
                                      : R, G, B)
#endif
        for (unsigned j = 0; j < col; ++j)
        {
            // const uchar * const uc_pixel = ptr;
            R += ptr[0];
            B += ptr[1];
            G += ptr[2];
            ptr += 3;
        }
#if defined(OMP)
    }
#endif
#endif

#if defined(GRAYWORLD)
    //Average
    // #pragma omp ordered
    R /= ((row * col)/2);
    B /= ((row * col)/2);
    G /= ((row * col)/2);
    //Calculate the average value of the three RGB channels of the image
    double GrayValue = (R + G + B) / 3;
    //Calculate the gain coefficient of the three channels
    double kr = GrayValue / R;
    double kg = GrayValue / G;
    double kb = GrayValue / B;
#if defined(DEBUG)
    printf("GrayValue %.5f\n", GrayValue);
    printf("Average B, G, R:%.5f %.5f %.5f\n", B, G, R);
    printf("kb, kg, kr: %.5f %.5f %.5f\n", kb, kg, kr);
#endif

#endif
    cout << "\nTEST " << int(col) << "," << int(row) << endl;
    //-- According to the diagonal model of Von Kries. For each pixel in the image C adjustment adjusts its RGB components
    //-- After that filter using euclidean distance;
    static uint32_t i, j;
    // uint16_t r_p, b_p, g_p;
    static uint8_t r = 0; //*p++; //src r
    static uint8_t g = 0; //*p++; //src b
    static uint8_t b = 0;

    static uint8_t *src_ptr;
    static uint8_t *dst_ptr;
    static uint8_t *dst2_ptr;

    src_ptr = src->ptr<uint8_t>(0);
    dst_ptr = dst->ptr<uint8_t>(0);
#if defined(DEBUG_PC)
    dst2_ptr = dst2->ptr<uint8_t>(0);
#endif

#if defined(OMP)
    omp_set_num_threads(NUM_THR);

#if defined(GRAYWORLD)
#pragma omp parallel num_threads(NUM_THR) firstprivate(dst_ptr, src_ptr) private(j, r, g, b) shared(col, src, kr, kg, kb, euclidean_distance, euclidean_color, lookupTable_new)
    {
#else
#pragma omp parallel num_threads(NUM_THR) firstprivate(dst_ptr, src_ptr) private(j, r, g, b) shared(col, src, euclidean_distance, euclidean_color, lookupTable_new)
// #pragma omp parallel default(none) num_threads(NUM_THR) private(j, r, g, b) shared(col, src_ptr, dst_ptr, euclidean_distance, euclidean_color, lookupTable_new) 
    {
#endif
// #pragma omp for 
// schedule(static) nowait
#endif
        for (j = 0; j < col; j++)
        {
            //get R G B values
            r = src_ptr[0];
            g = src_ptr[1];
            b = src_ptr[2];
            src_ptr += 3;

#if defined(GRAYWORLD)
            r = (uint8_t)((kr * r) > 255 ? 255 : kr * r);
            g = (uint8_t)((kg * g) > 255 ? 255 : kg * g);
            b = (uint8_t)((kb * b) > 255 ? 255 : kb * b);
#endif

#if defined(DEBUG_PC)
            dst2_ptr[0] = r;
            dst2_ptr[1] = g;
            dst2_ptr[2] = b;
            dst2_ptr += 3;
#endif
            // *dst_ptr = lookupTable[((r / 10) << 10) + ((g / 10) << 5) + (b / 10)];
            // *dst_ptr = lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE];
            *dst_ptr = lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10];

            dst_ptr += 1;
        }
#if defined(OMP)

    }
#endif
}

static __inline__ Point filterWorld(Mat *src, Mat *dst)
{

    if (euclidean_distance < 1)
    {
        ROS_INFO("Euclidean distance should be grated than 1 but was given %d", euclidean_distance);
        return Point(0, 0);
    }

    row = src->rows;
    col = src->cols;
    static int i = 0, j = 0;
#if defined(DEBUG_PC)
    dst2 = (Mat *)&cv_ptr_out_rgb.image;
#endif

#if defined(DEBUG_PC)
    if (src->isContinuous() && dst2->isContinuous() && dst->isContinuous())
#else
    if (src->isContinuous() && dst->isContinuous())
#endif
    {
        printf("It's always continous");
        col = col * row;
        row = 1;
        continous_method(src, dst);
    }
    else
    {
        normal_method(src, dst);
    }

    // #if defined(OMP)
    //     }
    // #endif
    morphologyEx(*dst, *dst, MORPH_OPEN, getStructuringElement(MORPH_CROSS, Size(3, 3)));
    morphologyEx(*dst, *dst, MORPH_CLOSE, getStructuringElement(MORPH_CROSS, Size(3, 3)));
    vector<vector<Point>> contours;
    findContours(*dst, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.size() == 0)
    {
#if defined(DEBUG)
        cout << "0 contours" << endl;
#endif
        return Point(0, 0);
    }
    std::sort(contours.begin(), contours.end(), cmp);

    // vector<Point> max_contour = *max_element(contours.begin(), contours.end(), cmp);
    if (contourArea(contours[0]) < contour_area_min) // this is for full resolution
    {
#if defined(DEBUG)
        cout << "Didnt find any" << endl;
#endif
        return Point(0, 0);
    }
#if defined(DEBUG_PC)
    drawContours(*dst2, contours, 0, Scalar(euclidean_color.at(0), euclidean_color.at(1), euclidean_color.at(2)), 2, LINE_8, 0, 0);
#endif

    vector<vector<Point>> list_exits(5); //list_exits = {"South": [], "North": [], "West": [], "East": [], "rest of the points":[]}

    double epsilon = 0.01 * arcLength(contours[0], false); //eps_line
    vector<Point> aprox;
    approxPolyDP(contours[0], aprox, epsilon, true);
    for (i = 0; i < aprox.size(); i++)
    {
        Point point = aprox[i];
        if (point.y > row - pix_off)
        { // point on south border
#if defined(DEBUG_PC)
            cv::circle(*dst2, point, 5, Scalar(0, 255, 0), 10);
#endif
            add_in_group_by_distance(&list_exits[0], point);
        }
        else if (point.y < pix_off)
        { // point on north border
#if defined(DEBUG_PC)
            cv::circle(*dst2, point, 5, Scalar(0, 255, 255), 10);
#endif
            add_in_group_by_distance(&list_exits[1], point);
        }
        else if (point.x < pix_off)
        { // point on west border
#if defined(DEBUG_PC)
            cv::circle(*dst2, point, 5, Scalar(0, 0, 255), 10);
#endif
            add_in_group_by_distance(&list_exits[2], point);
        }
        else if (point.x > col - pix_off)
        { // point on east border
#if defined(DEBUG_PC)
            cv::circle(*dst2, point, 5, Scalar(255, 255, 0), 10);
#endif
            add_in_group_by_distance(&list_exits[3], point);
        }
        else
        {
#if defined(DEBUG_PC)
            cv::circle(*dst2, point, 5, Scalar(255, 0, 0), 10);
#endif
            list_exits[4].insert(list_exits[4].end(), point);
        }
    }
    int num_exits = 0;
    for (i = 0; i < list_exits.size() - 1; i++) // Only count South,North,East,West
    {
        num_exits += list_exits[i].size();
    }
#if defined(DEBUG)
    cout << "Num exits " << num_exits << endl;
#endif
    point_to_go.x = 0;
    point_to_go.y = 0;
    if (num_exits > 2 and (last_turn.x == 0 && last_turn.y == 0))
    {
#if defined(DEBUG)
        cout << "choose the closest to the middle of the screen" << endl;
#endif
        //choose the min,with euclidean distance
        for (i = 0; i < list_exits.size(); i++)
        {
            for (j = 0; j < list_exits[i].size(); j++)
            {
                if ((point_to_go.x == 0 && point_to_go.y == 0) || calcDistEuclidean(Point(col / 2, row / 2), list_exits[i][j]) < calcDistEuclidean(Point(col / 2, row / 2), point_to_go))
                {
                    point_to_go = list_exits[i][j];
                }
            }
        }
        last_turn = point_to_go;
    }
    else if (last_turn.x != 0 || last_turn.y != 0)
    {
#if defined(DEBUG)
        cout << "choose the closest to the previous chosen point (for one last time)" << endl;
#endif
        //choose the min,with euclidean distance
        for (i = 0; i < list_exits.size(); i++)
        {
            for (j = 0; j < list_exits[i].size(); j++)
            {
                if ((point_to_go.x == 0 && point_to_go.y == 0) || calcDistEuclidean(last_turn, list_exits[i][j]) < calcDistEuclidean(last_turn, point_to_go))
                {
                    point_to_go = list_exits[i][j];
                }
            }
        }
        last_turn.x = 0;
        last_turn.y = 0;
    }
    else if (num_exits > 2 && list_exits[4].size() > 0)
    {
#if defined(DEBUG)
        cout << "choose the intersection " << endl;
#endif
        point_to_go = list_exits[4][0];
        last_turn = point_to_go;
    }
    else
    {
#if defined(DEBUG)
        cout << "choose the highest seen" << endl;
#endif
        for (i = 0; i < list_exits.size(); i++)
        {
            for (j = 0; j < list_exits[i].size(); j++)
            {
                if ((point_to_go.x == 0 && point_to_go.y == 0) || (point_to_go.y > list_exits[i][j].y))
                {
                    point_to_go = list_exits[i][j];
                }
            }
        }
    }
#if defined(DEBUG_PC)
    cv::circle(*dst2, robot_view_point, 0, Scalar(255, 255, 255), 25);
    cv::circle(*dst2, point_to_go, 5, Scalar(0, 0, 0), 25);
#endif

    return point_to_go;
}

void camera_info(const sensor_msgs::ImageConstPtr &msg)
{
    row = msg->height;
    col = msg->width;
    pix_off = msg->height / 10;

    cv_ptr_out = []
    {
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
#if defined(DEBUG_PC)

    cv_ptr_out_rgb = []
    {
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
#endif

    sub2 = n->subscribe("/usb_cam_node/image_raw", 1, image_data);
    image_pub = n->advertise<sensor_msgs::Image>("mybot_out/im_out", 1);

#if defined(DEBUG_PC)
    image_pub_rgb = n->advertise<sensor_msgs::Image>("mybot_out/im_out_rgb", 1);
#endif
    ROS_INFO("Params initialization ended. Starting main program");
    sub.shutdown(); //erase this function's subscribtion
}

void image_data(const sensor_msgs::ImageConstPtr &msg)
{
#if defined(DEBUG)
    static struct timeval beg, end;
    static double elapsed_time;
#endif
    int param;
    vector<int> eucl_c;
    static Point point_to_go;
    mybot_msg::navigate_msg msg_send;

    if (n->getParam("euclidean_distance", param) && param != euclidean_distance)
    {
        euclidean_distance = param;
        params_changed = true;
    }

    if (ros::param::has("euclidean_color") || ros::param::has("/euclidean_color"))
    {
        n->getParam("euclidean_color", eucl_c);
        if (euclidean_color.at(0) != (uint8_t)eucl_c.at(0) || euclidean_color.at(1) != (uint8_t)eucl_c.at(1) || euclidean_color.at(2) != (uint8_t)eucl_c.at(2))
        {
            euclidean_color.at(0) = (uint8_t)eucl_c.at(0);
            euclidean_color.at(1) = (uint8_t)eucl_c.at(1);
            euclidean_color.at(2) = (uint8_t)eucl_c.at(2);
            params_changed = true;
        }
    }

    if (n->getParam("contour_area_min", param) && param != contour_area_min)
    {
        contour_area_min = param;
        params_changed = true;
    }

    if (n->getParam("luminocity_up", param) && param != luminocity_up)
    {
        luminocity_up = param;
        params_changed = true;
    }
    if (n->getParam("luminocity_low", param) && param != luminocity_low)
    {
        luminocity_low = param;
        params_changed = true;
    }
#ifdef LOOKUP
    if (params_changed)
    {
        ROS_INFO("Params changed. Generating LUT!");
        generate();
        ROS_INFO("Generating LUT finished");
        params_changed = false;
    }
#endif
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8); // modify shared msg,be careful with other callbacks of same topic
#if defined(DEBUG)
        gettimeofday(&beg, NULL);
#endif
        point_to_go = filterWorld((Mat *)&cv_ptr->image, (Mat *)&cv_ptr_out.image);

        msg_send.header.stamp = ros::Time::now();
        msg_send.header.frame_id = "/navigate";

        cout << point_to_go << endl;
        if (point_to_go.x != 0 && point_to_go.y != 0)
        { // we found a line

#if defined(DEBUG_PC)
            Mat *dst2 = (Mat *)&cv_ptr_out_rgb.image;
            cv::circle(*dst2, robot_view_point, 0, Scalar(255, 255, 255), 25);
#endif

            float angle = angle_between_points(robot_view_point, point_to_go);
            if (angle > 90)
            {
#if defined(DEBUG)
                cout << "Right ";
#endif
                angle = -180 + angle;
            }
            else
            {
#if defined(DEBUG)
                cout << "Left ";
#endif
            }
#if defined(DEBUG)
            cout << angle << endl;
#endif
            // left side: 0->90 from left to the center
            // right side: -90->0 from center to the right

            // publish angle to turn for the brain_node
            msg_send.has_line = true;
            msg_send.angle = angle;
        }
        else
        {
            msg_send.has_line = false;
            msg_send.angle = 0;
        }
#if defined(DEBUG)
        gettimeofday(&end, NULL);

        // compute and print the elapsed time in millisec
        elapsed_time = (end.tv_sec - beg.tv_sec) * 1000.0;    // sec to ms
        elapsed_time += (end.tv_usec - beg.tv_usec) / 1000.0; // us to ms

        fps_counter.insert(fps_counter.begin(), double(1000 / elapsed_time)); // insert element at start
        fps_counter.pop_back();

        printf("Time/frame: %f ms, %f fps: \n", elapsed_time,
               (double)(std::accumulate(fps_counter.begin(), fps_counter.end(),
                                        decltype(fps_counter)::value_type(0)) /
                        fps_counter.size()));
#endif

        cv_ptr_out.header.seq = seq++;
        cv_ptr_out.header.stamp = ros::Time::now();
        chatter_pub.publish(msg_send);
        image_pub.publish(cv_ptr_out.toImageMsg());
#if defined(DEBUG_PC)
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

int main(int argc, char *argv[])
{

    cout << sizeof(lookupTable_new) << endl;
    ros::init(argc, argv, "talker");
    ros::NodeHandle no;
    n = &no;

    chatter_pub = no.advertise<mybot_msg::navigate_msg>("/custom_msg", 1);

    sub = no.subscribe("/usb_cam_node/image_raw", 1, camera_info);
    ROS_INFO("Waiting for image_raw... (at least one publish)");
    // ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/usb_cam_node/image_raw");
    ros::spin();

    return 0;
}

#ifdef LOOKUP
// Generate lookuptable for sqrt values of single
// precision for comparison with euclidean/lum
// till 255 with 154 being the max(euclidean/lum)
void generate()
{
    cout << "Generating " << int(euclidean_color.at(0)) << "," << int(euclidean_color.at(1)) << "," << int(euclidean_color.at(2)) << " : " << int(euclidean_distance) << endl;
    for (int r = 0; r < 256; r += 10)
    {
        for (int g = 0; g < 256; g += 10)
        {
            for (int b = 0; b < 256; b += 10)
            {
#if defined(HSV)
                double h, s, v;
                uint8_t hsv_min, hsv_max;
                uint8_t delta;

                // MinMax() Needs at least 4 comparisons
                hsv_min = min_3(r, g, b);
                hsv_max = max_3(r, g, b);
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
                unsigned int temp = custom_sqrt((0.299 * (r - euclidean_color.at(0)) * (r - euclidean_color.at(0))) + (0.587 * (g - euclidean_color.at(1)) * (g - euclidean_color.at(1))) + (0.144 * (b - euclidean_color.at(2)) * (b - euclidean_color.at(2))));
#elif defined(RGB) && defined(MYSQRT)
                unsigned int temp = custom_sqrt((r - euclidean_color.at(0)) * (r - euclidean_color.at(0)) + (g - euclidean_color.at(1)) * (g - euclidean_color.at(1)) + (b - euclidean_color.at(2)) * (b - euclidean_color.at(2)));
#elif defined(RGB)
                unsigned int temp = sqrt((r - euclidean_color.at(0)) * (r - euclidean_color.at(0)) + (g - euclidean_color.at(1)) * (g - euclidean_color.at(1)) + (b - euclidean_color.at(2)) * (b - euclidean_color.at(2)));
#endif

#if defined(RGB) and defined(HSV) and defined(CUSTOM)
                if ((r < luminocity_low && g < luminocity_low && b < luminocity_low) || (r > luminocity_up && g > luminocity_up && b > luminocity_up))
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 0;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 0;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 0;
                }
                else if (!inRange(0, 100, h / 2) && temp < euclidean_distance)
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 255;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 255;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 255;
                }
#elif defined(RGB) and defined(HSV)
                if (!inRange(0, 100, h / 2) && temp < euclidean_distance)
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 255;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 255;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 255;
                }
#elif defined(HSV) and defined(CUSTOM)
                if ((r < luminocity_low && g < luminocity_low && b < luminocity_low) || (r > luminocity_up && g > luminocity_up && b > luminocity_up))
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 0;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 0;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 0;
                }
                else if (!inRange(0, 100, h / 2))
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 255;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 255;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 255;
                }
#elif defined(HSV)
                if (!inRange(0, 100, h / 2))
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 255;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 255;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 255;
                }
#elif defined(RGB) and defined(CUSTOM)
                if ((r < luminocity_low && g < luminocity_low && b < luminocity_low) || (r > luminocity_up && g > luminocity_up && b > luminocity_up))
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 0;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 0;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 0;
                }
                else if (temp < euclidean_distance)
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 255;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 255;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 255;
                }
#elif defined(RGB)
                if (temp < euclidean_distance)
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 255;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 255;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 255;
                }
#endif
                else
                {
                    //                     lookupTable2[r / COLOR_DECREASE][g / COLOR_DECREASE][b / COLOR_DECREASE] = 0;
                    lookupTable_new[(r / 10 * (25 + 1) * (25 + 1)) + (g / 10 * (25 + 1)) + b / 10] = 0;
                    //lookupTable[(r/10 << 10) + (g/10 << 5) + b/10] = 0;
                }
            }
        }
    }
}
#endif

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
        const uint8_t *src_pic_point = (uint8_t *)(src_pic->imageData + y * src_pic->widthStep);

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
