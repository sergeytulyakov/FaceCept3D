#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "LeafNode.h"

using namespace cv;
using namespace std;
using std::vector;

namespace hpe
{


    void evaluatePackedTree(Mat patchInt, Mat maskInt, Mat treeMat, Mat treeIdxMap, int initDepth, const cv::Rect roi, LeafNode *leaf)
    {

        int leafDim = treeMat.cols;//(int*) mxGetDimensions(input[2]);
        leaf->mean.create(1, 6, CV_64FC1);
        int node = initDepth;
        double pnode = (int)treeMat.at<double>(node, 1);
        int incr = 0;

        int xa1, ya1, xa2, ya2, xb1, yb1, xb2, yb2;

        while (pnode == -1) {
            xa1 = roi.x + (int)treeMat.at<double>(node, 2);
            ya1 = roi.y + (int)treeMat.at<double>(node, 3);
            xa2 = xa1 + (int)treeMat.at<double>(node, 6);
            ya2 = ya1 + (int)treeMat.at<double>(node, 7);
            xb1 = roi.x + (int)treeMat.at<double>(node, 4);
            yb1 = roi.y + (int)treeMat.at<double>(node, 5);
            xb2 = xb1 + (int)treeMat.at<double>(node, 8);
            yb2 = yb1 + (int)treeMat.at<double>(node, 9);

            double numrA = patchInt.at<double>(ya2, xa2) - patchInt.at<double>(ya2, xa1) - patchInt.at<double>(ya1, xa2) + patchInt.at<double>(ya1, xa1);
            double numA = maskInt.at<double>(ya2, xa2) - maskInt.at<double>(ya2, xa1) - maskInt.at<double>(ya1, xa2) + maskInt.at<double>(ya1, xa1);

            double numrB = patchInt.at<double>(yb2, xb2) - patchInt.at<double>(yb2, xb1) - patchInt.at<double>(yb1, xb2) + patchInt.at<double>(yb1, xb1);
            double numB = maskInt.at<double>(yb2, xb2) - maskInt.at<double>(yb2, xb1) - maskInt.at<double>(yb1, xb2) + maskInt.at<double>(yb1, xb1);

            double mxA = (numA > 1) ? numA : 1;
            double mxB = (numB > 1) ? numB : 1;

            double testVal = numrA / mxA - numrB / mxB;
            int test = (testVal >= treeMat.at<double>(node, 11)) ? 1 : 0;

            node = (int)treeMat.at<double>(node, 0) - 1;
            incr = node + test + 1;
            node = (int)(treeIdxMap.at<double>(node + incr, 0) - 1);
            pnode = (int)treeMat.at<double>(node, 1);
        }

        /*for(int i=0; i< leafDim; i++)
        {
            *out = treeMat.at<double>(node, i);
            out++;
        }*/

        leaf->pfg = treeMat.at<double>(node, 2);
        leaf->trace = treeMat.at<double>(node, 3);
        //cout << treeMat.at<double>(node, 4) << endl;
        leaf->mean = (Mat_<double>(1, 6) << treeMat.at<double>(node, 4),
                      treeMat.at<double>(node, 5),
                      treeMat.at<double>(node, 6),
                      treeMat.at<double>(node, 7),
                      treeMat.at<double>(node, 8),
                      treeMat.at<double>(node, 9));
        /*leaf->mean.at<double>(0) = treeMat.at<double>(node, 4);
        leaf->mean.at<double>(1) = treeMat.at<double>(node, 5);
        leaf->mean.at<double>(2) = treeMat.at<double>(node, 6);
        leaf->mean.at<double>(3) = treeMat.at<double>(node, 7);
        leaf->mean.at<double>(4) = treeMat.at<double>(node, 8);
        leaf->mean.at<double>(5) = treeMat.at<double>(node, 9);*/

        //return leafPtr;
    }

}