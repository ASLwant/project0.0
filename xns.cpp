#include "c.cpp"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;
const char* dev = "/dev/ttyS0";
int main() {
    uint8_t buff[512];
    serialPort myserial;
    int  nread, nwrite;
    cout << "serialPort Test" << endl;
    myserial.OpenPort(dev);
    myserial.setup(4800, 0, 8, 1, 'N');
    nread = myserial.readBuffer(buff, 1);
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Error opening video stream or file" << endl;
        return -1;
    }

    namedWindow("Color Detection", WINDOW_AUTOSIZE);

    Mat frame, hsvFrame;
    Mat redMask, blueMask, combinedMask;

    int num;
    num = buff[0];

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, hsvFrame, COLOR_BGR2HSV);


        Scalar lowerRed = Scalar(0, 127, 128);
        Scalar upperRed = Scalar(10, 255, 255);
        Scalar lowerBlue = Scalar(110, 50, 254);
        Scalar upperBlue = Scalar(115, 255, 255);

        //得到1/3筛选出红色
        if (num == 1||num == 3) {
            inRange(hsvFrame, lowerRed, upperRed, redMask);
            bitwise_not(redMask, blueMask);
            combinedMask = redMask;
        }//得到2/4筛选出蓝色
        else if (num == 2||num ==4) {
            inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
            bitwise_not(blueMask, redMask);
            combinedMask = blueMask;
        }
        vector<vector<Point>> contours;
        findContours(combinedMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//查找轮廓


        for (const auto& contour : contours) {

            double area = contourArea(contour);

            if (area > 100) {
                Rect rect = boundingRect(contour);
                rectangle(frame, rect.tl(), rect.br(), Scalar(0, 255, 0), 2);//画矩形
                vector<double> Dimensions(Rect rect_ion);//pnp位置解算
                {
                    Mat cameraMatrix = (Mat_<double>(3, 3) << 542.5448628971645, 0, 308.6090582512243, 0, 556.6297658902104, 67.84247127318397, 0, 0, 1);
                    Mat distCoeffs = (Mat_<double>(5, 1) << 0.1030398998730105, -0.0166943280189579, -0.08003425365739675, 0.001095242436811841, -0.02006035890323307);

                    vector<Point2f> image = vector<Point2f>
                    {

                    Point2f(rect.x,rect.y),
                    Point2f(rect.x + rect.width,rect.y),
                    Point2f(rect.x + rect.width,rect.y + rect.height),
                    Point2f(rect.x,rect.y + rect.height)
                    };
                    vector<Point3f> obj = vector<Point3f>
                    {
                        Point3f(-2.5f,-2.5f,0),
                        Point3f(2.5f,-2.5f,0),
                        Point3f(2.5f,2.5f,0),
                        Point3f(-2.5f,2.5f,0)

                    };
                    vector<double> rvec;
                    vector<double> tvec;
                    solvePnP(obj, image, cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
                    if (num == 1 || num == 2) {
                        myserial.writeBuffer(buff, tvec[0] + tvec[1] + tvec[2]);
                    }
                    else if (num == 3 || num == 4) {
                    Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
                    circle(frame, center, 5, Scalar(255, 0, 0), -1);
                    myserial.writeBuffer(buff, center.x + center.y);
                    }
                }
            }
        }
        imshow("Color Detection", frame);


        char key = waitKey(1);
    };

    cap.release();
    destroyAllWindows();
    return 0;
}
