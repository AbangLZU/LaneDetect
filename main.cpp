#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "laneDetection.h"
#include "calibration.h"

using namespace std;

cv::VideoCapture laneVideo;
cv::Mat videoFrame; // Video Frame.
cv::Mat videoFrameUndistorted; // Video Frame (after calibration).
cv::Mat videoFramePerspective; // Video Frame (after perspective transform).
cv::Mat _videoFrameUndistorted;
cv::Size videoSize; // Input Variable Size.
cv::Mat cameraMatrix, dist; //Calibration Matrix.
cv::Mat perspectiveMatrix; //Homography Matrix.
cv::Point2f perspectiveSrc[] = {cv::Point2f(565,470), cv::Point2f(721,470), cv::Point2f(277,698), cv::Point2f(1142,698)};
cv::Point2f perspectiveDst[] = {cv::Point2f(300,0), cv::Point2f(980,0), cv::Point2f(300,720), cv::Point2f(980,720)};

int main(int argc, char **argv)
{

    //Get the Perspective Matrix.
    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);


    //Load the video.
    if(argc < 2)
    {
        cerr << "There is no input video." << endl;
        return -1;
    }

    laneVideo.open(argv[1]); //Load the video.
    if(!laneVideo.isOpened())
    {
        cerr << "Could not open the video." << endl;
        return -1;
    }
    videoSize = cv::Size((int)laneVideo.get(cv::CAP_PROP_FRAME_WIDTH),
                     (int)laneVideo.get(cv::CAP_PROP_FRAME_HEIGHT));


    //--------------Camera Calibration Start-----------------
    cv::FileStorage fsRead;
    fsRead.open("Intrinsic.xml", cv::FileStorage::READ);
    cv::Mat src = cv::imread("../camera_cal/calibration2.jpg");
    cv::Mat dst;
    if (fsRead.isOpened() == false)
    {
        CameraCalibrator myCameraCalibrator;
        myCameraCalibrator.doCalibration(cameraMatrix, dist);
        cv::FileStorage fs;
        fs.open("Intrinsic.xml", cv::FileStorage::WRITE);
        fs << "CameraMatrix" << cameraMatrix;
        fs << "Dist" << dist;
        fs.release();
        fsRead.release();
        cout << "There is no existing intrinsic parameters XML file." << endl;
        cout << "Start calibraton......" << endl;
    }
    else
    {
        fsRead["CameraMatrix"] >> cameraMatrix;
        fsRead["Dist"] >> dist;
        fsRead.release();
    }
    undistort(src, dst, cameraMatrix, dist);
    //--------------Camera Calibration Finish-----------------

    cv::Mat warpEdge;
    cv::Mat imageRedChannel;
    cv::Mat redBinary;
    cv::Mat mergeImage;
    cv::Mat histImage;
    cv::Mat maskImage;
    cv::Mat warpMask;
    cv::Mat debugWindowROI;
    cv::Mat resizePic;

    //===========Start Real Time Processing===========
    float laneDistant = 0;
    stringstream ss;
    cv::namedWindow("Real Time Execution", CV_WINDOW_NORMAL);
    laneVideo.set(cv::CAP_PROP_POS_FRAMES, 0);
    laneVideo >> videoFrame;

    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();

    cv::Mat showVideos(videoFrame.size().height, videoFrame.size().width, CV_8UC3, cv::Scalar(0,0,0));

    laneDetection LaneAlgoVideo(_videoFrameUndistorted, perspectiveMatrix);
    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();


    while(!videoFrame.empty())
    {

        clock_t start_time = clock();

        //Start Homography
        warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

        //Applying lane detection algorithm
        LaneAlgoVideo.laneDetctAlgo();

        //Detect the distance to lane center.
        laneDistant = LaneAlgoVideo.getLaneCenterDist();
        if(laneDistant > 0)
        {
            std::cout << abs(laneDistant) << "m " << " To the Right" << std::endl;
        }
        else
        {
            std::cout << abs(laneDistant) << "m " << " To the Left"<< std::endl;
        }

        mergeImage = LaneAlgoVideo.getFinalResult();
        imshow("Real Time Execution", mergeImage);

        clock_t end_time = clock();
        std::cout<<"time is "<< end_time - start_time<<std::endl;

        laneVideo >> videoFrame;
        if(videoFrame.empty()) break;

        //Calibration
        undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
        _videoFrameUndistorted = videoFrameUndistorted.clone();
        LaneAlgoVideo.setInputImage(_videoFrameUndistorted);

        if(cv::waitKey(10) == 27) break;
    }
    //===========Finish Real Time Processing===========

    cv::waitKey(0);
    return 0;

}


