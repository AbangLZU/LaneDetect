//
// Created by adam on 18-7-17.
//

#include "calibration.h"

//------------------------------------------------
// Load the image files for camera calibration
//------------------------------------------------
void CameraCalibrator::setFilename()
{
    m_filenames.clear();
    m_filenames.push_back("../camera_cal/calibration1.jpg");
    m_filenames.push_back("../camera_cal/calibration2.jpg");
    m_filenames.push_back("../camera_cal/calibration3.jpg");
    m_filenames.push_back("../camera_cal/calibration4.jpg");
    m_filenames.push_back("../camera_cal/calibration5.jpg");
    m_filenames.push_back("../camera_cal/calibration6.jpg");
    m_filenames.push_back("../camera_cal/calibration7.jpg");
    m_filenames.push_back("../camera_cal/calibration8.jpg");
    m_filenames.push_back("../camera_cal/calibration9.jpg");
    m_filenames.push_back("../camera_cal/calibration10.jpg");
    m_filenames.push_back("../camera_cal/calibration11.jpg");
    m_filenames.push_back("../camera_cal/calibration12.jpg");
    m_filenames.push_back("../camera_cal/calibration13.jpg");
    m_filenames.push_back("../camera_cal/calibration14.jpg");
    m_filenames.push_back("../camera_cal/calibration15.jpg");
    m_filenames.push_back("../camera_cal/calibration16.jpg");
    m_filenames.push_back("../camera_cal/calibration17.jpg");
    m_filenames.push_back("../camera_cal/calibration18.jpg");
    m_filenames.push_back("../camera_cal/calibration19.jpg");
    m_filenames.push_back("../camera_cal/calibration20.jpg");
}


//------------------------------------------------
//Detect chessboard points
//------------------------------------------------
void CameraCalibrator::addPoints()
{
    vector<cv::Point2f> chessboardCorner;
    vector<cv::Point3f> realWorldCoord;
    cv::Mat image;
    //real wrold coordinates
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<9; j++)
        {
            realWorldCoord.push_back(cv::Point3f(i, j, 0.0f));
        }
    }

    //find chessboard 2D coordinates

    for(int i=0; i<m_filenames.size(); i++)
    {
        image = cv::imread(m_filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
        m_imageSize = image.size();
        findChessboardCorners(image, cv::Size(9,6), chessboardCorner);
        if(chessboardCorner.size() == 54)
        {
            m_dstPoints.push_back(realWorldCoord);
            m_srcPoints.push_back(chessboardCorner);
        }
    }
}



//------------------------------------------------
// Start Calibration
//------------------------------------------------
void CameraCalibrator::doCalibration(cv::Mat &cameraMatrix, cv::Mat &dist)
{
    setFilename();
    addPoints();

    vector<cv::Mat> rvecs, tvecs;
    calibrateCamera(m_dstPoints,
                    m_srcPoints,
                    m_imageSize,
                    cameraMatrix,
                    dist,
                    rvecs,
                    tvecs);

}
