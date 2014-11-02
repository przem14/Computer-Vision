#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <vector>
#include "src/Calibrator.h"

using namespace cv;
using namespace std;

const int board_dt = 20;
int n_boards;
int board_h;
int board_w;

int findAllCorners(
        Mat image,
        int board_n,
        Size board_sz,
        VideoCapture capture,
        vector<vector<Point2f> > &image_points,
        vector<vector<Point3f> > &object_points);

void findCornersOnBoard(
        Mat image,
        Mat gray_image,
        Size board_sz,
        vector<Point2f> corners,
        int &successes,
        int board_n,
        vector<vector<Point2f> > &image_points,
        vector<vector<Point3f> > &object_points);

void findCornersOnBoard(
        Mat image,
        Mat gray_image,
        Size board_sz,
        vector<Point2f> corners,
        int &successes,
        int board_n,
        vector<vector<Point2f> > &image_points,
        vector<vector<Point3f> > &object_points)
{
    bool found = findChessboardCorners(
                    image,
                    board_sz,
                    corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

    if (found)
    {
        cvtColor(image, gray_image, CV_BGR2GRAY);
        cornerSubPix(
            gray_image,
            corners,
            Size(11,11),
            Size(-1,-1),
            TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

        drawChessboardCorners(
            image,
            board_sz,
            corners,
            found);
    }
    imshow("Calibration", image);
    waitKey(33);

    if (corners.size() == board_n)
    {
        for(int j = 0; j < board_n; ++j)
        {
            image_points[successes][j]    = corners[j];
            object_points[successes][j].x = j / board_w;
            object_points[successes][j].y = j % board_w;
            object_points[successes][j].z = 0.0f;
        }
        successes++;
    }
}

int findAllCorners(
        Mat image,
        int board_n,
        Size board_sz,
        VideoCapture capture,
        vector<vector<Point2f> > &image_points,
        vector<vector<Point3f> > &object_points)
{
    vector<Point2f> corners;
    Mat gray_image(image.size(), CV_8UC1);
    int successes = 0, frame = 0;

    while(successes < n_boards)
    {
        if(frame++ % board_dt == 0)
        {
            cout << "Successes: " << successes << "\n";
            findCornersOnBoard(
                    image,
                    gray_image,
                    board_sz,
                    corners,
                    successes,
                    board_n,
                    image_points,
                    object_points);
        }

        int c = -1;
        if(c == 'p')
        {
            c = 0;
            while(c != 'p' && c != 27)
                c = waitKey(250);
        }
        capture >> image;
    }
    return successes;
}

int main(int argc, char* argv[])
{
    if(argc != 4)
    {
        cerr << "ERROR: Wrong number of input parameters\n";
        return -1;
    }

    board_w = atoi(argv[1]);
    board_h = atoi(argv[2]);
    n_boards = atoi(argv[3]);
    int board_n = board_w * board_h;
    Size board_sz = Size(board_w, board_h);

	Calibrator calibrator(n_boards, board_w, board_h);

    vector<vector<Point2f> >  image_points(n_boards, vector<Point2f>(board_n));
    vector<vector<Point3f> > object_points(n_boards, vector<Point3f>(board_n));
    Mat intrinsicMatrix(3, 3, CV_32FC1);
    Mat distortionCoeffs(5, 1, CV_32FC1);

    Mat image;
    VideoCapture capture(0);
    capture >> image;

    int successes = findAllCorners(
                        image,
                        board_n,
                        board_sz,
                        capture,
                        image_points,
                        object_points);

    intrinsicMatrix.at<float>(0,0) = 1.0f;
    intrinsicMatrix.at<float>(1,1) = 1.0f;

    vector<Mat> rot;
    vector<Mat> trans;

    calibrateCamera(
            object_points,
            image_points,
            image.size(),
            intrinsicMatrix,
            distortionCoeffs,
            rot,
            trans);

    FileStorage fs_write("Calibration_params.xml", FileStorage::WRITE);
    fs_write << "intr" << intrinsicMatrix << "dist" << distortionCoeffs;
    fs_write.release();

    calibrator.showImages(intrinsicMatrix, distortionCoeffs);

    return 0;
}
