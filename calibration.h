//
// Created by adam on 18-7-17.
//

#ifndef LANE_DETECT_CALIBRATION_H
#define LANE_DETECT_CALIBRATION_H


#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace std;

class CameraCalibrator
{
private:
    vector<string> m_filenames;
    vector<vector<cv::Point2f> > m_srcPoints;
    vector<vector<cv::Point3f> > m_dstPoints;
    cv::Size m_imageSize;
public:
    void setFilename();
    void addPoints();
    void doCalibration(cv::Mat &cameraMatrix, cv::Mat &dist);
};



#endif //LANE_DETECT_CALIBRATION_H
