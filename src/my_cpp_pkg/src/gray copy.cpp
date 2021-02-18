#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <sys/time.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include "geometry_msgs/Bool.h"
#include <sstream>

// from cv_bridge import CvBridge, CvBridgeError
// from nav_msgs.msg import Odometry
// from sensor_msgs.msg import Image, LaserScan, Range,Imu
// from tf.transformations import euler_from_quaternion


// #include <cstdio.h>
#include <cmath>
#define RASPI
#define DEBUG

using namespace cv;
using namespace std;

class ImageProccessor{

    int line_width = 100;
    int row;
    int col;
    uchar tD=10;
    uchar tU=200;
    uchar pix_off;  // trial and error
    double eps = 0.1;
    Point intersection_turn = Point(0, 0); // x=y=0 == NULL for us
    Point point_to_go;
    ros::Subscriber sub;
    ros::NodeHandle n;
    static cv_bridge::CvImageConstPtr cv_ptr;


    public:ImageProccessor(int row,int col){
        this->row = row;
        this->col = col;
        this->pix_off = row/5;
        this->sub = this->n.subscribe("/usb_cam_node/image_raw", 1,  &ImageProccessor::image_data);
        
    }
    public:ImageProccessor(int row,int col,int line_width){
        this->row = row;
        this->col = col;
        this->pix_off = row/5;
        this->line_width = line_width;
        this->sub = n.subscribe("/usb_cam_node/image_raw", 1,  &ImageProccessor::image_data);
        
    }

    ~ImageProccessor(){
        destroyAllWindows();
    }

    static void image_data(const sensor_msgs::ImageConstPtr& msg)
   {
    struct timeval beg, end;
    double elapsed_time;
    Mat result;
      try
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        
        gettimeofday(&beg, NULL);
        result = ImageProccessor::GrayWorld(cv_ptr->image,Scalar(0,0,255),160,200,10);
        gettimeofday(&end, NULL);
        #ifndef RASPI
        imshow("Received", cv_ptr->image);
        #endif
      }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
    }

    double fastPow(double a, double b) {
        union {
            double d;
            int x[2];
        } u = { a };
        u.x[1] = (int)(b * (u.x[1] - 1072632447) + 1072632447);
        u.x[0] = 0;
        return u.d;
    }


    int calcDistEuclidean(Point a, Point b,bool complex=false){
        return sqrt(fastPow(a.x-b.x,2)+fastPow(a.y-b.y,2));   
    }

    static bool cmp(const vector<Point>& lhs, const vector<Point>& rhs)
    {
        return contourArea(lhs) < contourArea(rhs);
    }

    Point centroid(Point* a, Point *b){
        return Point(a->x+b->x/2,a->y+b->y);
    }

    Point centroid(vector<Point> list_points){
        int sumx=0;
        int sumy=0;
        for (int i=0;i<list_points.size();i++){
            sumx+=list_points[i].x;
            sumy+=list_points[i].y;
        }
        return Point(sumx/(list_points.size()>0?list_points.size():1),sumy/(list_points.size()>0?list_points.size():1));
    }
    
    void add_group_by_distance(vector<Point> *src, Point point){
        for(int i=0;i< (*src).size();i++){
            if(hypot((*src)[i].x-point.x,(*src)[i].y-point.y)<this->line_width){
                (*src)[i] = centroid(&(*src)[i], &point);
                return;
            }
        }
        (*src).insert((*src).end(), point);
    }

    public:Mat GrayWorld(Mat src,Scalar euclidean_color, uchar euclidean_distance,uchar tU, uchar tD) {
        //Use the vector object to separate the R, G, and B channels of the input image
        vector <Mat> bgr;
        split(src, bgr);
        double B = 0;
        double G = 0;
        double R = 0;
        int row = src.rows;
        int col = src.cols;
        //Create a Mat object of the same size as the input image
        // Mat dst(col, row, CV_8UC3);
        Mat dst(row, col, CV_8UC1);
        if (euclidean_distance<1) return dst;
        //Get the RGB value of each pixel of the input image
        uchar r, g, b;
        #pragma omp parallel for reduction(+:R,G,B) shared(src)
        for (int i = 0; i < row; ++i)
        {
            for (int j = 0; j < col; ++j)
            {   
                B += 1.0 * src.ptr<uchar>(i,j)[0];
                G += 1.0 * src.ptr<uchar>(i,j)[1];
                R += 1.0 * src.ptr<uchar>(i,j)[2];
            }
        }
        #pragma omp ordered
        //Average
        B /= (row * col);
        G /= (row * col);
        R /= (row * col);
        // printf("Average B, G, R:%.5f %.5f %.5f\n", B, G, R);
        //Calculate the average value of the three RGB channels of the image
        double GrayValue = (B + G + R) / 3;
        // printf("GrayValue %.5f\n", GrayValue);
        //Calculate the gain coefficient of the three channels
        double kr = GrayValue / R;
        double kg = GrayValue / G;
        double kb = GrayValue / B;
        // printf("kb, kg, kr: %.5f %.5f %.5f\n", kb, kg, kr);
        //-- According to the diagonal model of Von Kries. For each pixel in the image C adjustment adjusts its RGB components
        //-- After that filter using euclidean distance; 
        Mat pixel(3,1,CV_8U);
        uchar pixel_out[3];
        uchar* p;
        uchar* out;
        uchar temp;
        int i,j;
        // blur( src_gray, src_gray, Size(3,3) );
        medianBlur(src, src, 3);
        #pragma omp parallel private(i, j, temp, pixel_out) shared(dst, euclidean_distance) 
        {
            #pragma omp for collapse(2) 
            for (i = 0; i < row; i++) {

                // p =(uchar *) src.ptr<uchar>(i);  // point to first color in row
                // out =(uchar *) dst.ptr<uchar>(i);  // point to first color in row
                // #pragma omp for private(pixel)// schedule(dynamic)
                
                for (j = 0; j < col; j++) {


                    pixel_out[0]= src.ptr<uchar>(i, j)[0]; //*p++; //src b
                    pixel_out[1]= src.ptr<uchar>(i, j)[1]; //*p++; //src g
                    pixel_out[2]= src.ptr<uchar>(i, j)[2]; //*p++; //src r


                    pixel_out[0] = (int)(kb * pixel_out[0]) > 255 ? 255 : (int)(kb * pixel_out[0]);
                    pixel_out[1] = (int)(kg * pixel_out[1]) > 255 ? 255 : (int)(kg * pixel_out[1]);
                    pixel_out[2] = (int)(kr * pixel_out[2]) > 255 ? 255 : (int)(kr * pixel_out[2]);

                    // pixel_out[0] = (int)(kb * pixel_out[0]) > 255 ? 255 : (int)(kb * pixel_out[0]);
                    // pixel_out[1] = (int)(kg * pixel_out[1]) > 255 ? 255 : (int)(kg * pixel_out[1]);
                    // pixel_out[2] = (int)(kr * pixel_out[2]) > 255 ? 255 : (int)(kr * pixel_out[2]);

                    if ((pixel_out[0] > tU && pixel_out[1] > tU && pixel_out[2] > tU) || (pixel_out[0] < tU && pixel_out[1] < tU && pixel_out[2] < tU)){
                    // if (pixel_out[0]>thres && pixel_out[1]>thres && pixel_out[2]> thres){
                        dst.ptr<uchar>(i,j)[0]=0;
                        // pixel_out[0] = 0;
                        // pixel_out[1] = 0;
                        // pixel_out[2] = 0;
                    }else{
                        temp = (uchar)(sqrt(fastPow(pixel_out[0]-euclidean_color[0],2)+fastPow(pixel_out[1]-euclidean_color[1],2)+fastPow(pixel_out[2]-euclidean_color[2],2)));
                        if (temp < euclidean_distance){
                            dst.ptr<uchar>(i,j)[0]=255; // one channel
                            // pixel_out[0] = euclidean_color[0];
                            // pixel_out[1] = euclidean_color[1];
                            // pixel_out[2] = euclidean_color[2];               
                        }else{
                            dst.ptr<uchar>(i,j)[0]=0;
                            // pixel_out[0] = 0;
                            // pixel_out[1] = 0;
                            // pixel_out[2] = 0;
                        }
                    }
                    
                }
            }
        }
        morphologyEx(dst, dst, MORPH_OPEN, getStructuringElement(MORPH_CROSS,Size(3,3)));
        morphologyEx(dst, dst, MORPH_CLOSE, getStructuringElement(MORPH_CROSS, Size(3, 3)));
        vector<vector<Point>> contours;
        findContours(dst, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
        // vector<Point> max_contour = *max_element(contours.begin(),contours.end(),cmp);
        // if (contourArea(max_contour) < 5000){
        //     return dst;
        // }
        
        // vector<vector<Point>> list_exits(5);  //list_exits = {"South": [], "North": [], "West": [], "East": [], "rest of the points":[]}

        // double epsilon = 0.01 * arcLength(max_contour, false); //eps_line
        // vector<Point> aprox;
        // approxPolyDP(max_contour, aprox, epsilon, true);
        
        // for(int i=0;i<aprox.size();i++){
        //     Point point = aprox[i];
        //     if (point.y > row - pix_off)  // point on south border
        //         add_group_by_distance(&list_exits[0], point);
        //     else if (point.y < pix_off)  // point on north border
        //         add_group_by_distance(&list_exits[1], point);
        //     else if (point.x < pix_off) // point on west border
        //         add_group_by_distance(&list_exits[2], point);
        //     else if (point.x > col - pix_off)  // point on east border
        //         add_group_by_distance(&list_exits[3], point);
        //     else
        //         list_exits[4].insert(list_exits[4].end(),point);
        // }
        // int num_exits = 0;
        // for (int i=0; i<list_exits.size()-1;i++){ // Only count South,North,East,West
        //     num_exits += list_exits[i].size();
        // }
        // #ifdef DEBUG
        //     cout << "Num exits " << num_exits << endl;
        // #endif
        // if (num_exits > 2 and (intersection_turn.x==0 && intersection_turn.y==0)){
        //     #ifdef DEBUG
        //         cout<<"choose the closest to the previous chosen point"<<endl;
        //     #endif
            
        //     point_to_go.x=0;
        //     point_to_go.y=0;
        //     //choose the min,with euclidean distance
            
                    
        //     for (int i=0;i<list_exits.size();i++){
        //         for(int j=0;j<list_exits[i].size();j++){
        //             if((point_to_go.x==0 && point_to_go.y==0) or calcDistEuclidean(point_to_go,list_exits[i][j])){
        //                 point_to_go = list_exits[i][j];
        //             }
        //         }
        //     }
        //     intersection_turn = point_to_go;
        // }
        // else if (intersection_turn.x!=0 || intersection_turn.y!=0){
        //     #ifdef DEBUG
        //         cout << "choose the closest to the previous chosen point (for one last time)" << endl;
        //     #endif
        //     point_to_go.x=0;
        //     point_to_go.y=0;
        //     //choose the min,with euclidean distance
        //     for (int i=0;i<list_exits.size();i++){
        //         for(int j=0;j<list_exits[i].size();j++){
        //             if((point_to_go.x==0 && point_to_go.y==0) or calcDistEuclidean(point_to_go,list_exits[i][j])){
        //                 point_to_go = list_exits[i][j];
        //             }
        //         }
        //     }
        //     intersection_turn.x=0;
        //     intersection_turn.y=0;
        // }else if(list_exits[4].size()>0){
        //     #ifdef DEBUG
        //         cout << "choose the intersection" << endl;
        //     #endif
        //     point_to_go = list_exits[4][0];
        // }else{
        //     #ifdef DEBUG
        //         cout << "choose the highest" << endl;
        //     #endif
        //     point_to_go.x=0;
        //     point_to_go.y=0;
        //     for (int i=0;i<list_exits.size();i++){
        //         for(int j=0;j<list_exits[i].size();j++){
        //             if(point_to_go.y<list_exits[i][j].y){
        //                 point_to_go = list_exits[i][j];
        //             }
        //         }
        //     }
        // }
        
        return dst;
    }

    Mat GetSquareImage(const Mat& img, int target_width = 500 )
    {
        int width = img.cols,
        height = img.rows;

        Mat square = Mat::zeros( target_width, target_width, img.type() );

        int max_dim = ( width >= height ) ? width : height;
        float scale = ( ( float ) target_width ) / max_dim;
        Rect roi;
        if ( width >= height )
        {
            roi.width = target_width;
            roi.x = 0;
            roi.height = height * scale;
            roi.y = ( target_width - roi.height ) / 2;
        }
        else
        {
            roi.y = 0;
            roi.height = target_width;
            roi.width = width * scale;
            roi.x = ( target_width - roi.width ) / 2;
        }

        resize( img, square( roi ), roi.size() );

        return square;
    }

    void spin(){
        static Mat result;

        ros::Rate loop_rate(30);
        while(ros::ok()){
            
            gettimeofday(&beg, NULL);
            result = GrayWorld(this->cv_ptr->image,Scalar(0,0,255),160,200,10);
            gettimeofday(&end, NULL);

            // compute and print the elapsed time in millisec
            elapsed_time = (end.tv_sec - beg.tv_sec) * 1000.0;      // sec to ms
            elapsed_time += (end.tv_usec - beg.tv_usec) / 1000.0;   // us to ms

            printf("Time/frame: %f ms, %f fps: \n", elapsed_time, 1000/elapsed_time);
            if (result.empty())
            {
                cout << "Error! THE IMAGE IS EMPTY.." << endl;
                // return -1;
            }
            #ifndef RASPI
            else
            {

                // imshow("Origin", frame);
                // imshow("Result", result);
                ; //publish
            }
            #endif
        }
    }

};


int main(int argc, char* argv[])
{   
    
    ros::init(argc,argv,"talker");
    namedWindow("Received", WINDOW_AUTOSIZE);
    ImageProccessor proc(480,640);
    ros::spin();
    return 0;
    VideoCapture cap;
    if (argc==1){
        cap.open("test2.mp4",CAP_V4L2);
    }else{
        if (strncmp(argv[1], "/dev/video", strlen("/dev/video")) == 0) {
            cap.open(argv[1], CAP_V4L2);
            //cap.set(CAP_PROP_FRAME_WIDTH, 1280);
            //cap.set(CAP_PROP_FRAME_HEIGHT, 720);
            cap.set(CAP_PROP_FRAME_WIDTH, 640);
            cap.set(CAP_PROP_FRAME_HEIGHT, 480);
        }
        else
            cap.open(argv[1]);
    }

    int euclidean_distance = (argc>2)?atoi(argv[2]):160;
    int thres_up = (argc>3)?atoi(argv[3]):200;
    int thres_down = (argc > 4) ? atoi(argv[4]) : 20;
    //cout << "Threshold" << thres << endl;
    cout << "Euclidean distance " << euclidean_distance << endl;
    // cap.set(CAP_PROP_FRAME_HEIGHT, 1280);
    // cap.set(CAP_PROP_FRAME_WIDTH, 720);
    double fps = cap.get(CAP_PROP_FPS);
    cout << "Fps" << fps << endl;
    cout << "Height" << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "Width" << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
    Mat frame;
    cap >> frame;
    Mat result;
    // ImageProccessor proc(frame.rows,frame.cols);
    #ifndef RASPI
    namedWindow("Result", WINDOW_AUTOSIZE);
    namedWindow("Origin", WINDOW_AUTOSIZE);
    #endif
    // Timing code
    struct timeval beg, end;
    double elapsed_time;
    int skip=0;
    while(ros::ok()){
        cap >> frame;
        if(frame.empty()){
            cout << "Empty frame!"<< endl;
            continue;
        }
        if(skip>0){
            skip-=1;
            continue;
        }
        // normalize(frame,frame,0,255,NORM_MINMAX);

        #ifndef RASPI
        // imshow("Origin1", proc.GetSquareImage(frame, 500));
        imshow("Origin1", frame);
        #endif
        // cvtColor(frame, frame, COLOR_RGB2BGR); // capture = RGB
        
        gettimeofday(&beg, NULL);
        result = proc.GrayWorld(frame,Scalar(0,0,255),euclidean_distance,thres_up,thres_down);
        gettimeofday(&end, NULL);

        // compute and print the elapsed time in millisec
        elapsed_time = (end.tv_sec - beg.tv_sec) * 1000.0;      // sec to ms
        elapsed_time += (end.tv_usec - beg.tv_usec) / 1000.0;   // us to ms

        printf("Time/frame: %f ms, %f fps: \n", elapsed_time, 1000/elapsed_time);
        if (result.empty())
        {
            cout << "Error! THE IMAGE IS EMPTY.." << endl;
            return -1;
        }
        #ifndef RASPI
        else
        {

            // imshow("Origin", frame);
            // imshow("Result", result);
            imshow("Origin", proc.GetSquareImage(frame, 500));
            imshow("Result", proc.GetSquareImage(result, 500));
        }
        #endif
        skip=0;
        // skip = (skip!=0)?skip:(int)(1000/elapsed_time)%(int)(fps); //skip X frames for smoothness on video
        cout << "skip "<< skip << endl;
        char c=(char)waitKey(100);
        if(c == 27  || c ==25 || c=='q') break;
    }

    return 0;
}
