#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Tree.h"
#include "csvMat.h"

using namespace cv;
using namespace std;
using std::vector;

#include <fstream>

namespace hpe
{

    Tree readTree(string csvRootName, int treeIdx)
    {
        cout << "Reading tree no. " << treeIdx << " from: " << csvRootName << endl;
        string treeIdxStr;
        ostringstream convert;
        convert << treeIdx;
        treeIdxStr = convert.str();
        string specsFileName = csvRootName + "tree00" + treeIdxStr + "_specs.csv";
        Mat specsMat;
        specsMat.create(1, 6, CV_64FC1);
        specsMat = readCSVMat(specsFileName, 1, 6);

        Tree t;
        t.treeMat.create(int(specsMat.at<double>(0)), int(specsMat.at<double>(1)), CV_64FC1);
        t.idxMap.create(int(specsMat.at<double>(2)), 1, CV_64FC1);

        string matFileName = csvRootName + "tree00" + treeIdxStr + "_Mat.csv";
        string idxMapFileName = csvRootName + "tree00" + treeIdxStr + "_IdxMap.csv";

        t.treeMat = readCSVMat(matFileName, int(specsMat.at<double>(0)), int(specsMat.at<double>(1)));
        t.idxMap = readCSVMat(idxMapFileName, int(specsMat.at<double>(2)), 1);

        return t;
    }

}