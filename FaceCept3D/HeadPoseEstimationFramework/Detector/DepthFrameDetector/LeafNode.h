#ifndef LEAFNODE_H
#define LEAFNODE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

namespace hpe
{

    class LeafNode {

        public:

            LeafNode() { }
            ~LeafNode() {
                mean.release();
            }

            // Probability of belonging to head region
            double pfg;
            // mean vector
            cv::Mat mean;
            // trace of the covariance matrix
            double trace;

    };

//LeafNode* evaluatePackedTree(Mat patchInt, Mat maskInt, Mat treeMat, Mat treeIdxMap, int initDepth, const cv::Rect roi);
    void evaluatePackedTree(Mat patchInt, Mat maskInt, Mat treeMat, Mat treeIdxMap, int initDepth, const cv::Rect roi, LeafNode *leaf);

}

#endif