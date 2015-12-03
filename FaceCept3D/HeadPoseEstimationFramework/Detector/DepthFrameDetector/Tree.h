#pragma once
#ifndef TREE_H
#define TREE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

namespace hpe
{

    class Tree {

        public:

            Tree() { }
            ~Tree() { }

            Mat treeMat, idxMap;
    };

    Tree readTree(string csvRootName, int treeIdx);

}

#endif