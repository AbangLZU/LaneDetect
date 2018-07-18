//
// Created by adam on 18-7-17.
//

#ifndef LANE_DETECT_LANEDETECTION_H
#define LANE_DETECT_LANEDETECTION_H


#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "math.h"

//using namespace cv;
using namespace std;
using namespace Eigen;

class laneDetection
{
private:
    cv::Mat perspectiveMatrix;
    cv::Mat oriImage; //The original input image.
    cv::Mat wrapImageGray;
    cv::Mat edgeImage; // The result of applying canny edge detection.
    cv::Mat warpEdgeImage;
    cv::Mat warpOriImage;
    vector<cv::Mat> imageChannels;
    cv::Mat RedBinary;
    cv::Mat mergeImage;
    cv::Mat mergeImageRGB;
    cv::Mat histImage; //Histogram visualization.
    cv::Mat maskImage; //The curve image used to blend to the original image.
    cv::Mat maskImageWarp;
    cv::Mat finalResult;
    vector<int> histogram; //Histogram of the detected features
    vector<cv::Point2f> laneL;
    vector<cv::Point2f> laneR;
    vector<cv::Point2f> curvePointsL;
    vector<cv::Point2f> curvePointsR;
    int laneLcount;
    int laneRcount;
    int midPoint; //The mid position of the view.
    int midHeight;
    int leftLanePos; //The detected left lane boundary position.
    int rightLanePos; //The detected right lane boundary position.
    short initRecordCount; // To record the number of times of executions in the first 5 frames.
    const int blockNum; //Number of windows per line.
    int stepY; //Window moving step.
    const int windowSize; //Window Size (Horizontal).
    Vector3d curveCoefL; //The coefficients of the curve (left).
    Vector3d curveCoefR; //The coefficients of the curve (left).
    Vector3d curveCoefRecordL[5]; //To keep the last five record to smooth the current coefficients (left).
    Vector3d curveCoefRecordR[5]; //To keep the last five record to smooth the current coefficients (right).
    int recordCounter;
    bool failDetectFlag; // To indicate whether the road marks is detected succesfully.
    void calHist();
    void boundaryDetection();
    void laneSearch(const int &lanePos, vector<cv::Point2f> &_line, int &lanecount, vector<cv::Point2f> &curvePoints, char dir);
    bool laneCoefEstimate();
    void laneFitting();
public:
    laneDetection(cv::Mat _oriImage, cv::Mat _perspectiveMatrix);
    ~laneDetection();
    void laneDetctAlgo();
    cv::Mat getEdgeDetectResult();
    cv::Mat getWarpEdgeDetectResult();
    cv::Mat getRedChannel();
    cv::Mat getRedBinary();
    cv::Mat getMergeImage();
    cv::Mat getHistImage();
    cv::Mat getMaskImage();
    cv::Mat getWarpMask();
    cv::Mat getFinalResult();
    float getLaneCenterDist();
    void setInputImage(cv::Mat &image);
};


#endif //LANE_DETECT_LANEDETECTION_H
