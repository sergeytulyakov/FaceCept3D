#ifndef LOCALFUNCTIONS_H
#define LOCALFUNCTIONS_H

#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdint.h>
#include "opencv2/core/core.hpp"
#include "Tree.h"

using namespace std;
using namespace cv;

namespace hpe
{

    struct paramList
    {
        double maskThreshold;
        int stride;
        int patchSize;
        double headPatchProb;
        double headDiameter;
        double headPatchDistTh;
        double minVotes;
        double yawTrace;
        double tiltTrace;
        double ytDiameter;
    };

    struct headPoseInfo
    {
        double x, y, yaw, tilt;
    };

    void loadConfig(std::string filename, paramList *p, bool verbose = false);
    Rect getBoundingBox(const Mat &im3D, paramList p);
    double computeNorm(vector<double> vec1, vector<double> vec2);
    double computeMean(vector<double> vec);
    headPoseInfo estimateHeadAngles(Mat depthFrame, vector<Tree> cascade, paramList p);

}

#endif