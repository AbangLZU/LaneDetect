//
// Created by adam on 18-7-18.
//

#include "calibration.h"

//------------------------------------------------
// Load the image files for camera calibration
// 加载图片:20
//------------------------------------------------
void CameraCalibrator::setFilename()
{
    m_filenames.clear();
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/01.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/02.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/03.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/04.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/05.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/06.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/07.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/08.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/09.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/10.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/11.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/12.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/13.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/14.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/15.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/16.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/17.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/18.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/19.jpg");
    m_filenames.push_back("/home/sunyu/ClionProjects/Lane-Detect/camera_cal/20.jpg");
    std::cout << "Image load over" << std::endl;
}


//------------------------------------------------
//Detect chessboard points
// 检测棋盘
//------------------------------------------------
void CameraCalibrator::addPoints()
{
    vector<cv::Point2f> chessboardCorner;
    vector<cv::Point3f> realWorldCoord;
    cv::Mat image;
    //real wrold coordinates
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<6; j++)
        {
            realWorldCoord.push_back(cv::Point3f(i, j, 0.0f));
        }
    }

    //find chessboard 2D coordinates

    for(int i=0; i<m_filenames.size(); i++)
    {
        image = cv::imread(m_filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
        m_imageSize = image.size();
        findChessboardCorners(image, cv::Size(6,4), chessboardCorner);
        if(chessboardCorner.size() == 24)
        {
            m_dstPoints.push_back(realWorldCoord);
            m_srcPoints.push_back(chessboardCorner);
        }
    }
    std::cout << m_dstPoints.size() << std::endl;
    std::cout << m_srcPoints.size() << std::endl;
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

    //undistort(src, dst, cameraMatrix, dist);
    //dst = src;
}

