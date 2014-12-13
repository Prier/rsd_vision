// THIS IS THE FINAL CODE

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>

// Used for sending the transformation matrix
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

//Our self define msg file
#include <rsd_vision/bricks_to_robot.h>

#define projectName rsd_vision
#define visualize true

using namespace cv;
using namespace std;

//#define xy_hys 10 // Works for 12Hz of the belt
//Below thease individual hysterese for x and y works at 20Hz of the belt.
#define x_hys 10
#define y_hysP 10
#define y_hysM 30 //20 at 10 fps

#define FPS 0.1 // 10 FPS

#define maxMorph 20

// Maybee thise defines need to be redefined when interfacing to the MES-server, regarding some MES-standards.
#define BLUE_BRICK     "Blue"
#define RED_BRICK       "Red"
#define YELLOW_BRICK "Yellow"

#define BLUE_MIN_HEIGHT    30
#define BLUE_MAX_HEIGHT    60
#define RED_MIN_HEIGHT     65
#define RED_MAX_HEIGHT    100
#define YELLOW_MIN_HEIGHT 105
#define YELLOW_MAX_HEIGHT 140

#define MAX_WIDTH          60

///////////////////////////////////////////////////////////
// Defines for the pin hole model and
///////////////////////////////////////////////////////////
#define resizeScale 2
#define focalLengthX 1194.773485/resizeScale
#define focalLengthY 1192.665666/resizeScale
#define initialZ 0.25

///////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////
static const std::string OPENCV_WINDOW = "Image window";

// Lower and Upper Limit
int top = 100;
int bund = 350;

// Red
int hue_min_red = 1;
int hue_max_red = 21;
int saturation_min_red = 0;
int saturation_max_red = 255;
int value_min_red = 160;
int value_max_red = 255;

// Yellow
int hue_min_yellow = 0;     // Yep, full hue range, and then subtracting red to get yellow
int hue_max_yellow = 255;
int saturation_min_yellow = 0;
int saturation_max_yellow = 255;
int value_min_yellow = 200;
int value_max_yellow = 255;

// Blue
//int hue_min_blue = 50;
//int hue_max_blue = 255;
//int saturation_min_blue = 0;
//int saturation_max_blue = 72;
//int value_min_blue = 0;
//int value_max_blue = 255;

// Blue - test with inverted images
int hue_min_blue = 0;
int hue_max_blue = 50;
int saturation_min_blue = 40;
int saturation_max_blue = 255;
int value_min_blue = 0;
int value_max_blue = 255;

// Morphology trackbars
int erode_iterations_red = 3;    // 1 erosioner for at sikre at alt gult er væk
int dilate_iterations_red = 3;   // Tre dilates efter eriosion for at sikre få samme area igen til de røde klodsser
int erode_iterations_yellow = 3;
int dilate_iterations_yellow = 3;
int erode_iterations_blue = 3;
int dilate_iterations_blue = 3;

int erode_iterations = 0;
int dilate_iterations = 0;

double publish_frequency = 2;

//bricks_msg.speed = 7.0901871809;    // @10Hz
//double speedSend = 7.0901871809;

struct Brick
{
    Brick() : speed(0) {}
    Point2d center;
    double angle;
    double height;
    double width;
    string color;
    double speed;
    double prevTime;
    vector<double> brickSpeedVector;
};

vector<Brick> brickTracking;

///////////////////////////////////////////////////////////
// Class...
///////////////////////////////////////////////////////////
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    projectName::bricks_to_robot bricks_msg;
    ros::Publisher p_pub;

    public:
    ImageConverter():it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/legoDetection/output_video", 1);
        cv::namedWindow(OPENCV_WINDOW);

        // Here I added the publisher so we can publish the lego pse
        //p_pub = nh_.advertise<geometry_msgs::Pose>("lego_pose", 1);
        p_pub = nh_.advertise<projectName::bricks_to_robot>("lego_pose", 1);

    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    queue<ros::Time> timeQueue;
    queue<Mat> inputImageQueue;
    Mat inputImage;

    ///////////////////////////////////////////////////////////
    // My functions...
    ///////////////////////////////////////////////////////////

    void imageProcessing(ros::Time time)
    {
        ros::Rate rate(publish_frequency);

        // Here the time is initialized...
//        ros::Time tid;
//        tid = ros::Time::now();

        // Resize the image
        Size size(inputImage.cols/resizeScale,inputImage.rows/resizeScale);//the dst image size,e.g.100x100
        resize(inputImage,inputImage,size);//resize image

        // Create a ROI since the
        Mat img_cropped;
        int roi_x = 150; // We need to re-crop to 130 to detect brick at outer most left side of belt. But put some grip tape on belt.
        int roi_y = 0;
        int roi_width = inputImage.cols-roi_x-50;
        //int roi_width = inputImage.cols-(2*roi_x);
        //int roi_width = inputImage.cols-roi_x;
        //int roi_height = inputImage.rows - (2*roi_y);
        int roi_height = inputImage.rows - 100;

        inputImage(Rect(roi_x,roi_y,roi_width,roi_height)).copyTo(img_cropped);
        //imshow("Cropped image", img_cropped);

        //cout << "Size of image is: " << img_cropped.cols << " x " << img_cropped.rows << endl;
        // cols = 740, rows = 440.

        //Convert the image into hsv
        Mat hsvImage;
        cvtColor(img_cropped,hsvImage,CV_BGR2HSV);

        // Initial the trackbars for tweaking...
        CreateTrackBarRed();
        CreateTrackBarYellow();
        CreateTrackBarBlue();

        //Convert to binary image using thresholding with colorsegmentation
        Mat img_red;
        inRange(hsvImage,
                Scalar(hue_min_red, saturation_min_red, value_min_red),
                Scalar(hue_max_red, saturation_max_red, value_max_red),
                img_red);
        //imshow("Red segmentation", img_red);

        Mat img_yellow;
        inRange(hsvImage,
                Scalar(hue_min_yellow, saturation_min_yellow, value_min_yellow),
                Scalar(hue_max_yellow, saturation_max_yellow, value_max_yellow),
                img_yellow);
       //imshow("Yellow segmentation ernice", img_yellow);

        Mat img_blue;
        inRange(hsvImage,
                Scalar(hue_min_blue, saturation_min_blue, value_min_blue),
                Scalar(hue_max_blue, saturation_max_blue, value_max_blue),
                img_blue);

        // Show the inRange result for each image
        //imshow("Red segmentation", img_red);
        //imshow("Yellow segmentation",img_yellow);
        //imshow("Blue segmentation", img_blue);

        // A small HSV hack has been implemented to propper find blue bricks.
        // The HSV for the blue ones need to be inverted in the binary image, before adding to the red-yellow segmentated image.

        // Do the inversion of the blue image, since we did the HSV hack for the blue brick.
        Mat img_blue_neg;
        img_blue_neg = 255 - img_blue;

        //imshow("Pure img_blue_neg", img_blue_neg);
        //img_seg_blue = img_blue_neg - img_yellow;
        //imshow("Pure img_seg_blue", img_seg_blue);

        // And the final image is:
        Mat img_seg;
        img_seg = img_yellow + img_blue_neg;
        //imshow("Pure img_yellow", img_yellow);
        //imshow("Pure img_seg", img_seg);


        // Here we can see, that all the bricks is founded in the img_seg.
        // Now we need to do some morph to remove small noise pixels etc...

        //imshow("Final image before morph", img_seg);

        // Note: Since the total segmentated image, img_seg is much nicer now there is really no noise pixel.
        // However the blue brick still suffer from small holes and therefore we close thoese holes by
        // use som Closing. Not Opening. Opening was more nicer to use, if we wanted to get rid of small noise pixels.
        Mat img_morph;
        int erode_iterations = 3;
        int dilate_iterations = 3;
//        createTrackbar("Erode", "Final image after morph", &erode_iterations, maxMorph);
//        createTrackbar("Dilate", "Final image after morph", &dilate_iterations, maxMorph);
        img_morph = Opening(img_seg, erode_iterations, dilate_iterations);

        imshow("Final image after morph", img_morph);

        // Do segmentation
        vector< vector <Point> > contours;
        findContours(img_morph, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        // Center
        vector<Brick> sendBrick;
        vector<Brick> showBrick;

        // Note: If the brick is not found after the line, then check on the minArea and maxArea
        // Perhaps the limits is too tight and should be expanded a little.
        Point leftLowerPoint, rightLowerPoint, leftUpperPoint, rightUpperPoint;
        leftLowerPoint.x = 0;
        leftLowerPoint.y = top;
        rightLowerPoint.x = img_cropped.cols;
        rightLowerPoint.y = leftLowerPoint.y;

        leftUpperPoint.x = 0;
        leftUpperPoint.y = bund;
        rightUpperPoint.x = img_cropped.cols;
        rightUpperPoint.y = leftUpperPoint.y;

        // Below: This is only for single LEGO bricks.
        // If two LEGO bricks is touching each other, then the area is of course larger
        // This is at the moment not talking into account...

        findCenterAndAngle(contours, sendBrick, showBrick, true, leftLowerPoint.y, leftUpperPoint.y, time.toSec(), img_cropped.cols);

        // Draw the centerpoint in the image
        double x,y,roll,pitch,yaw;
        //initialZ = 0.25;                     // Tested to fid best for x,y 24/11-2014.This z value for getting the x and y
        roll = 17.03;                   // Roll is negligible depending on the u value in the image.
        pitch = 0.0;

        double alignmentOffset = 0.007;       // Approximately 1-0.5 cm offset of alignment when compairing origo to the centerpoint of the conveyor belt.
        //double fx = 1194.773485/resizeScale; // Info extracted from /camera/camera_info topic with ROS. Use rostopic echo /camera/camera_info
        //double fy = 1192.665666/resizeScale; // Info extracted from /camera/camera_info topic with ROS. Use rostopic echo /camera/camera_info

        for (int i = 0; i < sendBrick.size(); i++)
        {
            // Set the x and y using the Pin hole model
            x = GetXY(sendBrick[i].center.x, initialZ, focalLengthX, img_cropped.cols)+alignmentOffset;
            y = GetXY(sendBrick[i].center.y, initialZ, focalLengthY, img_cropped.rows);

            //cout << "The angle for red brick is: " << angle_red[i] << endl;

            // Here we send just x,y as float64 in the msg file.
            bricks_msg.x = x;
            bricks_msg.y = y;
            bricks_msg.speed = sendBrick[i].speed;

            /* Note about the z-value
              When looking at the data, which was remeassured, vally of the conveyorbelt is between
              200 and 225. So middle is around 215 cm, measured from the metal side of the conveyour belt
              that is closest to the Master computer.
              Looking at image where a meassurement tape was set in 215 cm, the measurement tape
              was a slithly off compaired to the (u,v)= 0,0 origo. The measurement tape was to the right side of the origo.
              Result of this is imagefile z215.png.
              Is on dropbox at /home/christian/Dropbox/E14/RSD/images/z215.png
              This offset is approiximate 1 cmm, i.e. measurement tape lies 1 cm too much to the right, compaired to the origo.
              So we have to subtract the offset, when compairing to the measurement data.
              The measuring data is available at dropbox, at
              /home/christian/Dropbox/E14/RSD/testZvalue.ods
            */

            // Here we just send the z value as float64 in the msg file
            bricks_msg.z = GetZValue(x, initialZ, alignmentOffset);

            // Set the yaw compaired to the orientation founded...
            yaw = sendBrick[i].angle;

            // Here we just just the roll, pitch and yaw as float64 in the msg file
            bricks_msg.roll = yaw; //roll;
            bricks_msg.pitch = 0.0;
            bricks_msg.yaw = 0.0;

            // Set the time into bricks_msg
            bricks_msg.header.stamp = time;

            // Set the time into bricks_msg
            bricks_msg.header.frame_id = sendBrick[i].color;

            // Pubish the yellow brick pose on a topic together with pose and time.
            p_pub.publish(bricks_msg);

            // Here we publish on the ROS topic, the pose.
            // Then we keep track of what we have sent...
            sendBrick.erase(sendBrick.begin() + i);

        }

        if (visualize)
        {
            for (int i = 0; i < showBrick.size(); i++)
            { 
                if(showBrick[i].color == BLUE_BRICK)
                    circle(img_cropped, showBrick[i].center, 5, Scalar(255, 0, 0), -1, 8, 0);
                else if (showBrick[i].color == RED_BRICK)
                    circle(img_cropped, showBrick[i].center, 5, Scalar(0, 0, 255), -1, 8, 0);
                else
                    circle(img_cropped, showBrick[i].center, 5, Scalar(0, 255, 255), -1, 8, 0);

                circle(img_cropped, showBrick[i].center, 10, Scalar(0, 0, 0), 1, 8, 0);
                putText(img_cropped, centerToString("Center : ", showBrick[i].center),    Point(showBrick[i].center.x, showBrick[i].center.y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Angle  : ", showBrick[i].angle),     Point(showBrick[i].center.x, showBrick[i].center.y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Height : ", showBrick[i].height),    Point(showBrick[i].center.x, showBrick[i].center.y + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
            }



            // Drawings stuff on the output RGB image
            line(img_cropped, leftLowerPoint, rightLowerPoint, Scalar(0, 255, 0), 2, 8, 0);
            line(img_cropped, leftUpperPoint, rightUpperPoint, Scalar(0, 255, 0), 2, 8, 0);
            Point center;
            center.x = img_cropped.cols/2;
            center.y = img_cropped.rows/2;
            circle(img_cropped, center, 5, Scalar(255, 255, 255), -1, 8, 0);
            circle(img_cropped, center, 8, Scalar(0, 0, 0), 0, 8, 0);

            imshow("Cropped image", img_cropped);

            createTrackbar("Lower Line", "Cropped image", &top, img_cropped.rows);
            createTrackbar("Upper Line", "Cropped image", &bund, img_cropped.rows);

            // Add this little wait for 1ms to be able to let OpenCV use the imshow function. IF this is avoided the imshow dont show
            // any images...
            waitKey(1);
        }
    }

    void imageVisualizer()
    {

    }

    float RandomNumber(float Min, float Max)
    {
        return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
    }

    vector<Mat> GetRGB(Mat image)
    {
        vector<Mat> rgb_channel;
        split(image, rgb_channel);
        return rgb_channel;
    }

    void CreateTrackBarRed()
    {
        createTrackbar("Hue min", "Red segmentation", &hue_min_red, 255);
        createTrackbar("Hue max", "Red segmentation", &hue_max_red, 255);
        createTrackbar("Saturation min", "Red segmentation", &saturation_min_red, 255);
        createTrackbar("Saturation max", "Red segmentation", &saturation_max_red, 255);
        createTrackbar("Value min", "Red segmentation", &value_min_red, 255);
        createTrackbar("Value max", "Red segmentation", &value_max_red, 255);
    }

    void CreateTrackBarYellow()
    {
        createTrackbar("Hue min", "Yellow segmentation", &hue_min_yellow, 255);
        createTrackbar("Hue max", "Yellow segmentation", &hue_max_yellow, 255);
        createTrackbar("Saturation min", "Yellow segmentation", &saturation_min_yellow, 255);
        createTrackbar("Saturation max", "Yellow segmentation", &saturation_max_yellow, 255);
        createTrackbar("Value min", "Yellow segmentation", &value_min_yellow, 255);
        createTrackbar("Value max", "Yellow segmentation", &value_max_yellow, 255);
    }

    void CreateTrackBarBlue()
    {
        createTrackbar("Hue min", "Blue segmentation", &hue_min_blue, 255);
        createTrackbar("Hue max", "Blue segmentation", &hue_max_blue, 255);
        createTrackbar("Saturation min", "Blue segmentation", &saturation_min_blue, 255);
        createTrackbar("Saturation max", "Blue segmentation", &saturation_max_blue, 255);
        createTrackbar("Value min", "Blue segmentation", &value_min_blue, 255);
        createTrackbar("Value max", "Blue segmentation", &value_max_blue, 255);
    }

    Mat Opening(Mat image, int erode_iterations, int dilate_iterations)
    {
        Mat element = (cv::Mat_<uchar>(3,3) <<      1,1,1,
                                                    1,1,1,
                                                    1,1,1);

//        Mat element1 = (cv::Mat_<uchar>(3,3) <<     0,1,0,
//                                                    1,1,1,
//                                                    0,1,0);

//        Mat element2 = (cv::Mat_<uchar>(3,3) <<     0,1,0,
//                                                    0,1,0,
//                                                    0,1,0);

//        Mat element3 = (cv::Mat_<uchar>(3,3) <<     0,0,0,
//                                                    1,1,1,
//                                                    0,0,0);

//        Mat element4 = (cv::Mat_<uchar>(3,3) <<     1,0,0,
//                                                    0,1,0,
//                                                    0,0,1);

//        Mat element5 = (cv::Mat_<uchar>(3,3) <<     0,0,1,
//                                                    0,1,0,
//                                                    1,0,0);

//        Mat element6 = (cv::Mat_<uchar>(3,3) <<     1,0,1,
//                                                    0,1,0,
//                                                    1,0,1);

//        Mat element7 = (cv::Mat_<uchar>(2,2) <<     1,1,
//                                                    1,1);

        //cout << "element is: " << "\n" <<element << endl;
        Mat morph_img;
        erode(image, morph_img, element, Point(-1,-1), erode_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
        dilate(morph_img, morph_img, element, Point(-1,-1), dilate_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
        return morph_img;
    }

    Mat Closing(Mat image, int erode_iterations, int dilate_iterations)
    {
        Mat morph_img;
        dilate(image, morph_img, Mat(), Point(-1,-1), dilate_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
        erode(morph_img, morph_img, Mat(), Point(-1,-1), erode_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
        return morph_img;
    }

    void findCenterAndAngle(vector<vector<Point> > &_contours,
                                vector<Brick> &_sendBrick, vector<Brick> &_showBrick,
                                bool _degrees, int _lowerLine, int _upperLine,
                                double nowTime, int imageSizeCol)
    {
        // minArea is setted for red to 1700. maxArea is not defined yet
        vector<RotatedRect> minRect( _contours.size() );
        Brick tempBrick;
        double timeDiff, tempSpeed, distY;
        bool notTracking;

        // Next image .... we get the speed by looking in the difference of position for each image.

        for (uint i = 0; i < _contours.size(); i++) // each img
        {
            // Only look for contours with area larger than 500
            if (contourArea(_contours[i]) < 500)
            {
                continue;
            }

            // Using the minAreaRect function to gain the width, height, center coordinate and area.
            minRect[i] = minAreaRect( Mat(_contours[i]) );


            // Now we ignore if two same colored LEGO bricks is touching each other
            // Because if this is true, the area is much bigger and we have setted the
            // maximum Area.

            // Track LegoBricks compaired to the height and width of the system

            if(minRect[i].size.height > minRect[i].size.width)
            {
                tempBrick.height = minRect[i].size.height;
                tempBrick.width = minRect[i].size.width;
                //cout << "The heigh of the brick is: " << minRect[i].size.height << endl;
                //ratio = double(minRect[i].size.height)/double(minRect[i].size.width);
                //cout << "The ratio height/width is: " << ratio << endl;
            }

            else
            {
                tempBrick.height = minRect[i].size.width;
                tempBrick.width = minRect[i].size.height;
                //cout << "The heigh of the brick is: " << minRect[i].size.width << endl;
            }

            if(tempBrick.width > MAX_WIDTH)
            {
                continue;
            }

            //cout << "The tempHeigh is:" << tempHeight << endl;

            // Only look for height that is between minHeihgt and maxHeight
            // Reason for only loook for height, and not use the width/height ratio is
            // because if e.g the red brick is lying on the side, the ratio width/height for red range also finds
            // blue normal placed bricks.

            //cout << tempHeight << endl;

            if((tempBrick.height > BLUE_MIN_HEIGHT) && (tempBrick.height < BLUE_MAX_HEIGHT))
            {
                tempBrick.color = BLUE_BRICK;
            }
            else if((tempBrick.height > RED_MIN_HEIGHT) && (tempBrick.height < RED_MAX_HEIGHT))
            {
                tempBrick.color = RED_BRICK;
            }
            else if((tempBrick.height > YELLOW_MIN_HEIGHT) && (tempBrick.height < YELLOW_MAX_HEIGHT))
            {
                tempBrick.color = YELLOW_BRICK;
            }

            // Now get the center and angle
            tempBrick.center = minRect[i].center;
            tempBrick.angle = minRect[i].angle;

            if ( (floor(minRect[i].size.height + 0.5)) < (floor(minRect[i].size.width + 0.5)) )
            {
                tempBrick.angle = tempBrick.angle + 180;
            }
            else
            {
                tempBrick.angle = tempBrick.angle + 90;
            }

            if (!_degrees)
            {
                tempBrick.angle = tempBrick.angle * M_PI / 180;
            }

            notTracking = true;
            for (uint j = 0; j < brickTracking.size(); j++)
            {
                if ((tempBrick.center.x <= brickTracking[j].center.x + x_hys) &&
                    (tempBrick.center.x >= brickTracking[j].center.x - x_hys) &&
                    (tempBrick.center.y <= brickTracking[j].center.y + y_hysP) &&
                    (tempBrick.center.y >= brickTracking[j].center.y - y_hysM) )
                {
                    notTracking = false;

                    distY = abs(GetXY(tempBrick.center.y, initialZ, focalLengthY, imageSizeCol) -
                                GetXY(brickTracking[j].center.y, initialZ, focalLengthY, imageSizeCol));
                    timeDiff = nowTime -  brickTracking[j].prevTime;
                    //timeDiff is not used due to problems with ros::Time
                    tempSpeed = distY / FPS;
                    brickTracking[j].brickSpeedVector.push_back(tempSpeed);
                    //cout << "Tracking: " << tempSpeed << endl;

                    brickTracking[j].center.x = tempBrick.center.x;
                    brickTracking[j].center.y = tempBrick.center.y;
                    brickTracking[j].prevTime = nowTime;

                    if ((tempBrick.center.y < _lowerLine)) //or (tempBrick.center.y > _upperLine))
                    {
                        sort(brickTracking[j].brickSpeedVector.begin(), brickTracking[j].brickSpeedVector.end());
                        brickTracking[j].speed = brickTracking[j].brickSpeedVector[floor(brickTracking[j].brickSpeedVector.size()/2)];

                        //cout << "Stop Tracking: " << brickTracking[j].brickSpeedVector.size() << endl;
                        //cout << "End Speed: " << brickTracking[j].speed << endl;
                        _sendBrick.push_back(brickTracking[j]);
                        brickTracking.erase(brickTracking.begin() + j);
                    }
                    break;
                }
            }

            if (notTracking)
            {
                if ((tempBrick.center.y > _lowerLine) and (tempBrick.center.y < _upperLine))
                {
                    //cout << "Begin Tracking!" << endl;
                    tempBrick.prevTime = nowTime;
                    brickTracking.push_back(tempBrick);
                }
            }

            if ((tempBrick.center.y > _lowerLine) and (tempBrick.center.y < _upperLine))
            {
                _showBrick.push_back(tempBrick);
            }
        }
    }

    string doubleToString( string info, double number )
    {
        stringstream ss;
        ss << info << number;
        return ss.str();
    }

    string centerToString( string info, Point center )
    {
        stringstream ss;
        ss << info << center;
        return ss.str();
    }

    double GetZValue(double x, double initialZValue, double alignOffset)
    {
        // Declare z and the offset
        double offsetZ = 0;
        double finalZ = 0;

        //cout << "x is this in meters, when entering the function: " << x << endl;
        //cout << "and the alignOffset is: " << alignOffset << endl;

        // Take care of the alignment offset, by moving the legoBrick in x space a little bit to the left.
        //x += alignOffset;

        //cout << "x is this in meters, when the offset has been subtracted: " << x << endl;

        // If x is positive, then we are in the origo + right side of the image.
        if (x>=0)
        {
            // Then we apply the formula, which was derived out from the testing data
            offsetZ = 0.2393333333*x - 0.00182222222222;
        }
        // Means that x is negative, then we are on the left side of the image
        else
        {
            //offsetZ = -0.238*x + 0.04746666667;
            offsetZ = -0.238*x + 0.00182222222222;
        }

        // The real z value is then what we used to calculate x and y, plus this little offset
        // The output z value is in meters still.

        if(offsetZ < 0)
        {
            finalZ = offsetZ + initialZValue;
        }
        else if (offsetZ == 0)
        {
            finalZ = initialZValue;
        }
        else
        {
            finalZ = initialZValue - offsetZ;
        }
        //cout << "z is finally: " << finalZ << endl;
        return finalZ;
    }

    // The pinhole model
    double GetXY(int uv, double z, double fxfy, int imageColRow)
    {
        // Initialize variables. This could be either x or y output. Depends on the input arguments.
        // See how the function is called...

        double xy;
        // With the input argument, it is possible to use the pin hole model as followed
        // for x and y
        xy = (uv - imageColRow/2)*z/fxfy;

        // And return the xy
        return xy;
    }

    ///////////////////////////////////////////////////////////
    // The image call back function
    ///////////////////////////////////////////////////////////

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Store the image from the webcam into inputImage
        Mat tempImage = cv_ptr->image;
        inputImageQueue.push(tempImage);
        ros::Time tempTime;
        timeQueue.push(tempTime.now());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "legoDetection");
    ImageConverter ic;
    ros::Rate looprate(20);
    //ros::spin();
//NOTE Below was using 90% cpu load WHY!!!!!!!! semp solution was to image process in the callback function.
    while(ros::ok)
    {
        if(!ic.inputImageQueue.empty())
        {
            ic.inputImage = ic.inputImageQueue.front();
            ic.inputImageQueue.pop();
            ic.imageProcessing(ic.timeQueue.front());
            ic.timeQueue.pop();
        }

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
