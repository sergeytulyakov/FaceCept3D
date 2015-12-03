#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include "localFunctions.h"
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include "Tree.h"
#include "LeafNode.h"

using namespace std;
using namespace cv;

namespace hpe
{

    void loadConfig(std::string filename, paramList *p, bool verbose)
    {
        ifstream in(filename);
        string dummy;

        if (in.is_open()) {

            in >> dummy;
            in >> p->maskThreshold;

            in >> dummy;
            in >> p->stride;

            in >> dummy;
            in >> p->patchSize;

            in >> dummy;
            in >> p->headPatchProb;

            in >> dummy;
            in >> p->headDiameter;

            in >> dummy;
            in >> p->headPatchDistTh;

            in >> dummy;
            in >> p->minVotes;

            in >> dummy;
            in >> p->yawTrace;

            in >> dummy;
            in >> p->tiltTrace;

            in >> dummy;
            in >> p->ytDiameter;

        }
        else {
            cerr << "File not found " << filename << endl;
        }
        in.close();

        if (verbose)
        {
            cout << endl << "------------------------------------" << endl;
            cout << "Parameter List:       " << endl << endl;
            cout << "maskThreshold:            " << p->maskThreshold << endl;
            cout << "stride:			  " << p->stride << endl;
            cout << "patchSize:                " << p->patchSize << endl;
            cout << "headPatchProb:            " << p->headPatchProb << endl;
            cout << "headDiameter:             " << p->headDiameter << endl;
            cout << "headPatchDistTh:          " << p->headPatchDistTh << endl;
            cout << "minNoVotes:               " << p->minVotes << endl;
            cout << "yawTrace:                 " << p->yawTrace << endl;
            cout << "tiltTrace:                " << p->tiltTrace << endl;
            cout << "yawTiltDiameter:          " << p->ytDiameter << endl;

            cout << endl << "------------------------------------" << endl << endl;
        }
    }

    Rect getBoundingBox(const Mat &im3D, paramList p) {

        Rect bbox;
        int min_x = im3D.cols;
        int min_y = im3D.rows;
        int max_x = 0;
        int max_y = 0;
        int p_width = p.patchSize;
        int p_height = p.patchSize;

        for (int y = 0; y < im3D.rows; y++)
        {
            const double *Mi = im3D.ptr<double>(y);
            for (int x = 0; x < im3D.cols; x++) {

                if (Mi[x] > 0) {

                    min_x = MIN(min_x, x);
                    min_y = MIN(min_y, y);
                    max_x = MAX(max_x, x);
                    max_y = MAX(max_y, y);
                }

            }
        }

        int new_w = max_x - min_x;
        int new_h = max_y - min_y;

        bbox.x = MIN(im3D.cols - 1, MAX(0 , min_x));
        bbox.y = MIN(im3D.rows - 1, MAX(0 , min_y));

        //int new_w = max_x - min_x + p_width;
        //int new_h = max_y - min_y + p_height;

        //bbox.x = MIN( im3D.cols-1, MAX( 0 , min_x - p_width/2 ) );
        //bbox.y = MIN( im3D.rows-1, MAX( 0 , min_y - p_height/2) );

        bbox.width  = MAX(0, MIN(new_w, im3D.cols - bbox.x));
        bbox.height = MAX(0, MIN(new_h, im3D.rows - bbox.y));

        return bbox;
    }

    double computeNorm(vector<double> vec1, vector<double> vec2)
    {
        double result = 0;
        int dim = vec1.size();
        vector<double> diff(dim);
        double sumSq = 0;
        for (int i = 0; i < dim; i++)
        {
            diff[i] = vec1[i] - vec2[i];
            sumSq += pow(diff[i], 2);
        }
        result = sqrt(sumSq);
        return result;
    }

    double computeMean(vector<double> vec)
    {
        double sum = 0;
        for (int k = 0; k < vec.size(); k++)
        {
            sum += vec[k];
        }
        return sum / vec.size();
    }

    Mat meanShift(Mat dataPts, double bandWidth, int *maxClustPointer)
    {
        *maxClustPointer = -1;
        int biggestClusterSize = 0;
        int numDim = dataPts.cols;
        int numPts = dataPts.rows;
        int numClust = -1;
        double bandSq = bandWidth * bandWidth;
        std::vector<int> initPtInds(numPts);
        for (int k = 0; k < initPtInds.size(); k++)
            initPtInds[k] = k;

        double stopThresh = 1e-3 * bandWidth;

        Mat clustCent(0, numDim, CV_64FC1);

        vector<bool> beenVisitedFlag(numPts);
        int numInitPts = numPts;
        //vector<int> clusterVotes(numPts);

        while (numInitPts > 0)
        {
            int tempInd = (int)std::ceil((float)(numInitPts - 1) * std::rand() / RAND_MAX);
            double stInd = (double)initPtInds[tempInd];

            std::vector<double> myMean(numDim);
            for (int i = 0; i < numDim; i++)
            {
                myMean[i] = (double)dataPts.at<double>(stInd, i);
            }

            vector<int> thisClusterVotes(numPts);

            while (1)
            {
                vector<double> sqDistToAll(numPts);
                for (int i = 0; i < numPts; i++)
                {
                    double result = 0;
                    for (int j = 0; j < numDim; j++)
                    {
                        double diff = (double)dataPts.at<double>(i, j) - myMean[j];
                        result += pow(diff, 2);
                    }
                    sqDistToAll[i] = result;
                }
                vector<bool> inInds(numPts);
                for (int i = 0; i < numPts; i++)
                {
                    inInds[i] = sqDistToAll[i] < bandSq;
                    if (inInds[i] == true)
                    {
                        thisClusterVotes[i]++;
                    }
                }

                int countTrues = count_if(inInds.begin(), inInds.end(), [](bool b) {
                    return b == true;
                });
                vector<double> myOldMean = myMean;
                cv::Mat tempData(countTrues, numDim, CV_64FC1);
                int localIdx = 0;
                for (int i = 0; i < inInds.size(); i++)
                {
                    if (inInds[i] == true)
                    {
                        for (int j = 0; j < numDim; j++)
                        {
                            tempData.at<double>(localIdx, j) = (double)dataPts.at<double>(i, j);
                        }
                        localIdx++;
                        beenVisitedFlag[i] = true;
                    }
                }
                Mat myMeanMat;
				cv::reduce(tempData, myMeanMat, 0, CV_REDUCE_AVG);
                myMeanMat.copyTo(myMean);

                // compute the 2-norm of the difference-between-means vector
                double meanNorm = computeNorm(myOldMean, myMean);

                if (meanNorm < stopThresh)
                {
                    int mergeWith = -1;
                    for (int cn = 0; cn <= numClust; cn++)
                    {
                        vector<double> localClustCent(numDim);
                        clustCent.row(cn).copyTo(localClustCent);
                        double distToOther = computeNorm(myMean, localClustCent);

                        if (distToOther < bandWidth / 2)
                        {
                            mergeWith = cn;
                            break;
                        }
                    }

                    if (mergeWith >= 0)
                    {
                        Mat localMat = 0.5 * (myMeanMat + clustCent.row(mergeWith));
                        localMat.copyTo(clustCent.row(mergeWith));
                    }
                    else
                    {
                        numClust++;
                        clustCent.push_back(myMeanMat);
                        if (tempData.rows > biggestClusterSize)
                        {
                            biggestClusterSize = tempData.rows;
                            *maxClustPointer = *maxClustPointer + 1;
                        }
                        //myMeanMat.copyTo(clustCent.row(numClust));
                    }
                    break;
                }
            }

            initPtInds.clear();
            for (int i = 0; i < beenVisitedFlag.size(); i++)
            {
                if (beenVisitedFlag[i] == false)
                {
                    initPtInds.push_back(i);
                }
            }
            numInitPts = initPtInds.size();
        }
        return clustCent;
    }

    headPoseInfo estimateHeadAngles(Mat depthFrame, vector<Tree> cascade, paramList p)
    {
        headPoseInfo eAngles;
        eAngles.x = 0;
        eAngles.y = 0;
        eAngles.tilt = -100;
        eAngles.yaw = -100;

        int p_width = p.patchSize;
        int p_height = p.patchSize;
        Rect bbox = getBoundingBox(depthFrame, p);

        //integral image of the depth frame
        int rows = depthFrame.rows;
        int cols = depthFrame.cols;
        Mat depthInt(rows + 1, cols + 1, CV_64FC1);
        integral(depthFrame, depthInt);

        //mask
        depthFrame.convertTo(depthFrame, CV_32FC1);
        Mat mask(rows, cols, CV_32FC1);
        mask.setTo(0);
        cv::threshold(depthFrame, mask, p.maskThreshold, 1, THRESH_BINARY);

        //integral image of the mask
        Mat maskInt(rows + 1, cols + 1, CV_64FC1);
        maskInt.setTo(0);
        integral(mask, maskInt);

        //defines the test patch
        Rect roi = Rect(0, 0, p_width, p_height);

        int min_no_pixels = p_width * p_height / 10;

        //vector<LeafNode*> leaves;
        LeafNode bufferLeaf;

        //process each patch
        int startX = std::max(2, bbox.x + 1 - p_width / 2 + 1);
        int endX = std::min(bbox.x + bbox.width - p_width / 2 + 1, maskInt.cols - p_width);
        int startY = std::max(2, bbox.y + 1 - p_height / 2 + 1);
        int endY = std::min(bbox.y + bbox.height - p_height / 2 + 1, maskInt.rows - p_height);
        vector<double> headPatchIdxX, headPatchIdxY;

        for (roi.y = startY; roi.y <= endY; roi.y += p.stride)
        {
            for (roi.x = startX; roi.x <= endX; roi.x += p.stride)
            {
                int x0 = roi.x;
                int y0 = roi.y;
                int x1 = x0 + p_width - 1;
                int y1 = y0 + p_height - 1;

                //discard if the patch is filled with data for less than 10%
                double areaSum = maskInt.at<double>(y1, x1) - maskInt.at<double>(y1, x0 - 1) - maskInt.at<double>(y0 - 1, x1) + maskInt.at<double>(y0, x0);
                if (areaSum / (p_width * p_height) <= 0.1)
                    continue;

                evaluatePackedTree(depthInt, maskInt, cascade[0].treeMat, cascade[0].idxMap, 0, roi, &bufferLeaf);

                // store only patches that are almost certain to belong to the head
                if (bufferLeaf.pfg > p.headPatchProb)
                {
                    headPatchIdxX.push_back((double)x0);
                    headPatchIdxY.push_back((double)y0);
                }
            }
        }

        if (headPatchIdxX.size() > 0)
        {
            Mat dataPtsX(headPatchIdxX);
            Mat dataPtsY(headPatchIdxY);
            Mat dataPts;
            hconcat(dataPtsX, dataPtsY, dataPts);

            int maxClustIdx = -1; // initialize the index of the biggest cluster (in case there are more than 1, it is useful to know which is the biggest)
            Mat clustCent = meanShift(dataPts, p.headDiameter, &maxClustIdx);
            vector<double> clustCentVec(dataPts.cols);
            clustCent.row(maxClustIdx).copyTo(clustCentVec);

            // once the center of the patches that "think" they belong to the head has been found
            // it's time to call the X and Y offset trees that will vote for the head center
            // and the Yaw/Tilt trees for the yaw/tilt predictions

            Mat votes(0, 8, CV_64FC1); // offsetX, offsetY, offsetZ(not used), yaw, tilt, roll(not used), traceYaw, traceTilt
            Mat dummyVote(1, 8, CV_64FC1);
            Mat headPatchDataMat(0, 2, CV_64FC1);
            Mat headPatchCenterAux(1, 2, CV_64FC1);

            for (roi.y = startY; roi.y <= endY; roi.y += p.stride)
            {
                for (roi.x = startX; roi.x <= endX; roi.x += p.stride)
                {
                    vector<double> currPoint(clustCentVec.size());
                    currPoint[0] = roi.x;
                    currPoint[1] = roi.y;

                    int x0 = roi.x;
                    int y0 = roi.y;
                    int x1 = x0 + p_width - 1;
                    int y1 = y0 + p_height - 1;

                    double areaSum = maskInt.at<double>(y1, x1) - maskInt.at<double>(y1, x0 - 1) - maskInt.at<double>(y0 - 1, x1) + maskInt.at<double>(y0, x0);
                    double localNorm = computeNorm(clustCentVec, currPoint);

                    if ((areaSum / (p_width * p_height) > 0.1) & (localNorm <= p.headPatchDistTh))
                    {
                        headPatchCenterAux.at<double>(0, 0) = (double)(x0 + p.patchSize / 2);
                        headPatchCenterAux.at<double>(0, 1) = (double)(y0 + p.patchSize / 2);
                        headPatchDataMat.push_back(headPatchCenterAux);
                        // offset on X
                        evaluatePackedTree(depthInt, maskInt, cascade[1].treeMat, cascade[1].idxMap, 0, roi, &bufferLeaf);
                        dummyVote.at<double>(0) = x0 + p.patchSize / 2 - bufferLeaf.mean.at<double>(0);
                        // offset on Y
                        evaluatePackedTree(depthInt, maskInt, cascade[2].treeMat, cascade[2].idxMap, 0, roi, &bufferLeaf);
                        dummyVote.at<double>(1) = y0 + p.patchSize / 2 - bufferLeaf.mean.at<double>(1);
                        dummyVote.at<double>(2) = 0;
                        // Yaw
                        evaluatePackedTree(depthInt, maskInt, cascade[3].treeMat, cascade[3].idxMap, 0, roi, &bufferLeaf);
                        dummyVote.at<double>(3) = bufferLeaf.mean.at<double>(3);
                        dummyVote.at<double>(5) = 0;
                        dummyVote.at<double>(6) = bufferLeaf.trace;
                        // Tilt
                        evaluatePackedTree(depthInt, maskInt, cascade[4].treeMat, cascade[4].idxMap, 0, roi, &bufferLeaf);
                        dummyVote.at<double>(4) = bufferLeaf.mean.at<double>(4);
                        dummyVote.at<double>(7) = bufferLeaf.trace;
                        votes.push_back(dummyVote);
                    }
                }
            }

            vector<double> mnHC(2);
            mnHC[0] = 0;
            mnHC[1] = 0;

            Mat votesFin(8, votes.rows, CV_64FC1);
            transpose(votes, votesFin);

            vector<double> offsetXVec(votesFin.cols), offsetYVec(votesFin.cols), predYawVec(votesFin.cols);
            vector<double> predTiltVec(votesFin.cols), traceYawVec(votesFin.cols), traceTiltVec(votesFin.cols);
            votesFin.row(0).copyTo(offsetXVec);
            votesFin.row(1).copyTo(offsetYVec);
            votesFin.row(3).copyTo(predYawVec);
            votesFin.row(4).copyTo(predTiltVec);
            votesFin.row(6).copyTo(traceYawVec);
            votesFin.row(7).copyTo(traceTiltVec);

            if (offsetXVec.size() > 0)
            {
                mnHC[0] = computeMean(offsetXVec);
                mnHC[1] = computeMean(offsetYVec);
                eAngles.x = mnHC[0];
                eAngles.y = mnHC[1];

                vector<double> distHeadCenter(headPatchDataMat.rows);
                vector<double> distHeadCenterIdx(headPatchDataMat.rows);
                vector<double> headPatchDataLocal(2);
                for (int k = 0; k < headPatchDataMat.rows; k++)
                {
                    headPatchDataMat.row(k).copyTo(headPatchDataLocal);
                    distHeadCenter[k] = computeNorm(headPatchDataLocal, mnHC);
                    distHeadCenterIdx[k] = k;
                }
                std::sort(distHeadCenterIdx.begin(), distHeadCenterIdx.end(), [&distHeadCenter](size_t i1, size_t i2) {
                    return distHeadCenter[i1] < distHeadCenter[i2];
                });
                vector<int> headCenterInd(offsetXVec.size());
                headCenterInd.assign(offsetXVec.size(), 0);
                if (offsetXVec.size() > p.minVotes)
                {
                    for (int k = 0; k < p.minVotes; k++)
                    {
                        headCenterInd[(int)distHeadCenterIdx[k]] = 1;
                    }
                }
                else
                {
                    headCenterInd.assign(offsetXVec.size(), 1);
                }

                vector<int> traceInd(offsetXVec.size());
                traceInd.assign(offsetXVec.size(), 0);

                for (int k = 0; k < offsetXVec.size(); k++)
                {
                    if ((traceYawVec[k] + traceTiltVec[k]) / 2 < p.yawTrace)
                    {
                        traceInd[k] = 1;
                    }
                }

                vector<int> allInds(traceInd.size());
                Mat yawTiltPoints(0, 2, CV_64FC1);
                Mat yawTiltDummy(1, 2, CV_64FC1);

                for (int k = 0; k < traceInd.size(); k++)
                {
                    allInds[k] = traceInd[k] * headCenterInd[k];
                    if (allInds[k] == 1)
                    {
                        yawTiltDummy.at<double>(0) = predYawVec[k];
                        yawTiltDummy.at<double>(1) = predTiltVec[k];
                        yawTiltPoints.push_back(yawTiltDummy);
                    }
                }
                //std::transform(headCenterInd.begin() + 1, headCenterInd.end(), traceInd.begin()+ 1, std::multiplies<int>());

                int maxClustIdx = -1;
                Mat clustCent = meanShift(yawTiltPoints, p.ytDiameter, &maxClustIdx);
                if (maxClustIdx != -1 && maxClustIdx < clustCent.rows)
                {
                    eAngles.yaw = clustCent.at<double>(maxClustIdx, 0);
                    eAngles.tilt = clustCent.at<double>(maxClustIdx, 1);
                }

                //clustCent.row(maxClustIdx).copyTo(eAngles);
            }
        }

        return eAngles;
    }

}
