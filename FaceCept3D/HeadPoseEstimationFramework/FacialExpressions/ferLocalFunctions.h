#ifndef FERLOCALFUNCTIONS_H
#define FERLOCALFUNCTIONS_H

#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdint.h>
#include "opencv2/core/core.hpp"

namespace fer
{

# define M_PI           3.14159265358979323846  /* pi */

    using namespace std;
    using namespace cv;

    struct paramList
    {
        int d;
        int patchSize;
        int stride;
        int featSize;
        int noPatches;
        int noChannels;
        int noTrees;
        Mat patchCoords;
        vector<Mat> patchFeats;
    };

    struct randomTree
    {
        vector<double> cutVar, cutValue, rightChild, leafVal;
    };

    struct headPoseInfo
    {
        double x, y, yaw, tilt;
    };

    vector<string> get_all_files_names_within_folder(string folder);

    Mat RV_readCSVMat(string csvFrameName, int lin, int col);
    void RV_readParamList(const string foldername, paramList *p);
    vector<vector<randomTree>> RV_readAllForests(string forestDir, paramList params);
    vector<double> RV_computeHist(vector<double> vec, int noBins);

    vector<double> RV_testForest(Mat localData, vector<randomTree> &localForest, paramList params, int numberOfClasses);

    Rect RV_getBoundingBox(const Mat &im3D, paramList p);
    double RV_computeNorm(vector<double> vec1, vector<double> vec2);
    double RV_computeMean(vector<double> vec);
    Mat RV_preprocessDepthFrame(Mat frameIn);
    void RV_featExtractionSingleFrame(Mat frame, paramList params, vector<double> *bgPerc, vector<Mat> &result);
    vector<Mat> RV_channelsExtractionSingleFrame(Mat frame, paramList params);
    Mat RV_computeDiffFeats(vector<Mat> imageInt, vector<Mat> maskInt, Mat localFeat);

    double RV_gaussPdf(double x, double mu, double sigma);
    int RV_imshow(string windowName, Mat img);

    Mat RV_lbp(const Mat &src);

}

#endif