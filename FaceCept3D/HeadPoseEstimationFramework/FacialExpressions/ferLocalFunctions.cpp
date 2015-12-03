#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdint.h>
#include "opencv2/core/core.hpp"
#include "ferLocalFunctions.h"
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <Windows.h>
#include <map>
#include <boost/filesystem.hpp>

namespace fer
{

    using namespace std;
    using namespace cv;

    std::string to_string(int val)
    {
        std::stringstream str;
        str << val;
        return str.str();
    }

    vector<string> get_all_files_names_within_folder(string folder)
    {
        vector<string> names;

        namespace fs = boost::filesystem;
        fs::directory_iterator end_iter;
        if (fs::exists(folder) && fs::is_directory(folder))
        {
            for (fs::directory_iterator dir_iter(folder) ; dir_iter != end_iter ; ++dir_iter)
            {
                if (fs::is_regular_file(dir_iter->status()))
                {
                }
            }
        }

        return names;
    }


    Mat RV_readCSVMat(string csvFrameName, int lin, int col)
    {
        Mat output(lin, col, CV_64FC1);
        ifstream inFile(csvFrameName);
        string line;
        int linenum = 0;
        while (getline(inFile, line))
        {
            linenum++;
            //cout << "\nLine #" << linenum << ":" << endl;
            istringstream linestream(line);
            string item;
            int itemnum = 0;
            while (getline(linestream, item, ','))
            {
                itemnum++;
                //cout << "Item #" << itemnum << ": " << item << endl;
                double temp = (double)atof(item.c_str());
                output.at<double>(linenum - 1, itemnum - 1) = (double)temp;
            }
        }
        return output;
    }

    void RV_readParamList(const string foldername, paramList *p)
    {
        cout << "Reading params from: " << foldername << "...";
        ifstream in(foldername + "config.txt");
        string dummy;

        if (in.is_open()) {

            in >> dummy;
            in >> p->d;

            in >> dummy;
            in >> p->patchSize;

            in >> dummy;
            in >> p->stride;

            in >> dummy;
            in >> p->featSize;

            in >> dummy;
            in >> p->noPatches;

            in >> dummy;
            in >> p->noChannels;

            in >> dummy;
            in >> p->noTrees;

        }
        else {
            cerr << "File not found " << foldername + "config.txt" << endl;
            exit(-1);
        }
        in.close();

        Mat dummyMat(p->d, 16, CV_64FC1);
        dummyMat.setTo(0);
        for (int i = 0; i < p->noPatches; i++)
        {
            p->patchFeats.push_back(dummyMat);
        }

        Mat paramsPatchCoords = RV_readCSVMat(foldername + "paramsPatchCoords.csv", p->noPatches, 2);
        p->patchCoords = paramsPatchCoords;

        for (int i = 0; i < p->noPatches; i++)
        {
            // rectangle coordinates are imported from Matlab as they are, which means they obey Matlab indexing (starting from 1)
            // this will be taken care of explicitly at feature computation stage ...
            Mat patchFeatsAux = RV_readCSVMat(foldername + "patchFeats/patchFeats_" + to_string(i + 1) + ".csv", p->d, 16);
            Mat patchFeatsAuxInt(patchFeatsAux.rows, patchFeatsAux.cols, CV_8UC1);
            patchFeatsAux.convertTo(patchFeatsAuxInt, CV_8UC1);
            p->patchFeats[i] = patchFeatsAuxInt;
        }

        cout << "done!" << endl;
        cout << endl << "------------------------------------" << endl;
        cout << "Parameter List:" << endl << endl;
        cout << "d:             " << p->d << endl;
        cout << "patchSize:     " << p->patchSize << endl;
        cout << "stride:        " << p->stride << endl;
        cout << "featSize:      " << p->featSize << endl;
        cout << "noPatches:     " << p->noPatches << endl;
        cout << "noChannels:    " << p->noChannels << endl;
        cout << endl << "------------------------------------" << endl << endl;
    }

    vector<vector<randomTree>> RV_readAllForests(string forestDir, paramList params)
    {
        vector<vector<randomTree>> allForests(params.noPatches, vector<randomTree>(params.noTrees));
        cout << "Reading forests from: " << forestDir << "...";
        for (int i = 0; i < params.noPatches; i++)
        {
            for (int j = 0; j < params.noTrees; j++)
            {
                Mat dummySize = RV_readCSVMat(forestDir + "patch_" + to_string(i + 1) + "_tree_" + to_string(j + 1) + "_size.csv", 1, 1);
                Mat dummyMat = RV_readCSVMat(forestDir + "patch_" + to_string(i + 1) + "_tree_" + to_string(j + 1) + ".csv", 4, (int)dummySize.at<double>(0, 0));
                allForests[i][j].cutVar.reserve(dummyMat.cols);
                allForests[i][j].cutValue.reserve(dummyMat.cols);
                allForests[i][j].rightChild.reserve(dummyMat.cols);
                allForests[i][j].leafVal.reserve(dummyMat.cols);
                dummyMat.row(0).copyTo(allForests[i][j].cutVar);
                dummyMat.row(1).copyTo(allForests[i][j].cutValue);
                dummyMat.row(2).copyTo(allForests[i][j].rightChild);
                dummyMat.row(3).copyTo(allForests[i][j].leafVal);
                //cout << "patch: " << i << " tree: " << j << endl;
            }
        }
        cout << "done!" << endl;
        return allForests;
    }

    vector<double> RV_computeHist(vector<double> vec, int noBins)
    {
        // supposedly vec has exactly noBins unique values
        const double eps = 1.0 / vec.size();
        vector<double> out(noBins);
        out.assign(noBins, 0);
        for (int i = 0; i < (int)vec.size(); i++)
        {
            out[(int)vec[i] - 1] += eps;
        }
        return out;
    }

    vector<double> RV_testForest(Mat localData, vector<randomTree> &localForest, paramList params, int numberOfClasses)
    {
        vector<double> votes(params.noTrees);
        votes.assign(params.noTrees, 0);
        for (int k = 0; k < params.noTrees; k++)
        {
            randomTree tree = localForest[k];
            int currNode = 0;
            while (tree.cutVar[currNode] > 0)
            {
                if (localData.at<double>(tree.cutVar[currNode] - 1) > tree.cutValue[currNode])
                {
                    currNode = tree.rightChild[currNode];
                }
                else
                {
                    currNode = tree.rightChild[currNode] - 1;
                }
            }
            votes[k] = tree.leafVal[currNode];
        }
        vector<double> probs = RV_computeHist(votes, numberOfClasses);
        return probs;
    }


    Rect RV_getBoundingBox(const Mat &im3D, paramList p) {

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

    double RV_computeNorm(vector<double> vec1, vector<double> vec2)
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

    double RV_computeMean(vector<double> vec)
    {
        double sum = 0;
        for (int k = 0; k < (int)vec.size(); k++)
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
        for (int k = 0; k < (int)initPtInds.size(); k++)
            initPtInds[k] = k;

        double stopThresh = 1e-3 * bandWidth;

        Mat clustCent(0, numDim, CV_64FC1);

        vector<bool> beenVisitedFlag(numPts);
        int numInitPts = numPts;
        //vector<int> clusterVotes(numPts);

        while (numInitPts > 0)
        {
            int tempInd = (int)std::ceil((numInitPts - 1 - 1e-6) * std::rand() / RAND_MAX);
            int stInd = initPtInds[tempInd];

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
                for (int i = 0; i < (int)inInds.size(); i++)
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
                reduce(tempData, myMeanMat, 0, CV_REDUCE_AVG);
                myMeanMat.copyTo(myMean);

                // compute the 2-norm of the difference-between-means vector
                double meanNorm = RV_computeNorm(myOldMean, myMean);

                if (meanNorm < stopThresh)
                {
                    int mergeWith = -1;
                    for (int cn = 0; cn <= numClust; cn++)
                    {
                        vector<double> localClustCent(numDim);
                        clustCent.row(cn).copyTo(localClustCent);
                        double distToOther = RV_computeNorm(myMean, localClustCent);

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
            for (int i = 0; i < (int)beenVisitedFlag.size(); i++)
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

    Mat RV_preprocessDepthFrame(Mat frame)
    {
        Mat output;
        double minVal = 100;
        double maxVal = -100;
        Point minLoc, maxLoc;

        for (int i = 0; i < frame.rows; i++)
        {
            for (int j = 0; j < frame.cols; j++)
            {
                if ((frame.at<double>(i, j) < minVal) & (frame.at<double>(i, j) != -100))
                {
                    minVal = frame.at<double>(i, j);
                    minLoc.x = i;
                    minLoc.y = j;
                }

                if (frame.at<double>(i, j) > maxVal)
                {
                    maxVal = frame.at<double>(i, j);
                    maxLoc.x = i;
                    maxLoc.y = j;
                }
            }
        }

        for (int i = 0; i < frame.rows; i++)
        {
            for (int j = 0; j < frame.cols; j++)
            {
                if (frame.at<double>(i, j) != -100)
                {
                    frame.at<double>(i, j) = 255 - (frame.at<double>(i, j) - minVal) / (maxVal - minVal) * 254;
                }
                else
                {
                    frame.at<double>(i, j) = 0;
                }
            }
        }

        transpose(frame, output);
        return output;
    }

    void RV_featExtractionSingleFrame(Mat frame, paramList params, vector<double> *bgPerc, vector<Mat> &featData)
    {
        featData.clear();
        featData.resize((unsigned int)params.noPatches);
        for (int k = 0; k < params.noPatches; k++)
        {
            Mat dummyMat(1, (int)(params.d * params.noChannels), CV_64FC1);
            dummyMat.setTo(0);
            featData[k] = dummyMat;
        }

        int rows = frame.rows;
        int cols = frame.cols;

        vector<Mat> channels = RV_channelsExtractionSingleFrame(frame, params);
        vector<Mat> channelsInt(params.noChannels);
        vector<Mat> maskInt(params.noChannels);

        for (int i = 0; i < params.noChannels; i++)
        {
            Mat localChannel = channels[i];
            Mat localChannelInt(rows + 1, cols + 1, CV_64FC1);
            localChannelInt.setTo(0);
            integral(localChannel, localChannelInt);

            Mat localMask(rows, cols, CV_32FC1);
            localMask.setTo(0);
            localChannel.convertTo(localChannel, CV_32FC1); // for some (probably) stupid reason,
            // m***** f***** "threshold" function does not work with CV_64FC1...
            threshold(localChannel, localMask, 0.001, 1, THRESH_BINARY);

            Mat localMaskInt(rows + 1, cols + 1, CV_64FC1);
            localMaskInt.setTo(0);
            integral(localMask, localMaskInt);
            channelsInt[i] = localChannelInt;
            maskInt[i] = localMaskInt;
        }

        for (int k = 0; k < params.patchCoords.rows; k++)
        {
            Mat frameAux(frame, Rect((int)params.patchCoords.at<double>(k, 1),
                                     (int)params.patchCoords.at<double>(k, 0),
                                     (int)params.patchSize,
                                     (int)params.patchSize));
            double countBG = 0;
            for (int i = 1; i < frameAux.rows; i++) // first row and column are not covered by the rectangular features
                // so yes, index should start from 1
            {
                for (int j = 1; j < frameAux.cols; j++) // same here
                {
                    if (frameAux.at<double>(i, j) == 0)
                    {
                        countBG++;
                    }
                }
            }
            (*bgPerc)[k] = countBG / (params.patchSize - 1) / (params.patchSize - 1);

            vector<Mat> localImageInt(params.noChannels);
            vector<Mat> localMaskInt(params.noChannels);
            for (int i = 0; i < params.noChannels; i++)
            {
                Mat dummyMat1(channelsInt[i], Rect((int)params.patchCoords.at<double>(k, 1) + 1,
                                                   (int)params.patchCoords.at<double>(k, 0) + 1,
                                                   (int)params.patchSize,
                                                   (int)params.patchSize));
                localImageInt[i] = dummyMat1;
                Mat dummyMat2(maskInt[i], Rect((int)params.patchCoords.at<double>(k, 1) + 1,
                                               (int)params.patchCoords.at<double>(k, 0) + 1,
                                               (int)params.patchSize,
                                               (int)params.patchSize));
                localMaskInt[i] = dummyMat2;
            }

            Mat feats = RV_computeDiffFeats(localImageInt, localMaskInt, params.patchFeats[k]);
            featData[k] = feats;
        }
        // return featData;
    }

    Mat RV_computeDiffFeats(vector<Mat> imageInt, vector<Mat> maskInt, Mat localFeat)
    {
        Mat feats(1, localFeat.rows * imageInt.size(), CV_64FC1);
        feats.setTo(0);

        //int numRectangles = params.patchFeats[0].cols / 4;
        for (int i = 0; i < (int)imageInt.size(); i++)
        {
            Mat localImg = imageInt[i];
            Mat localMask = maskInt[i];
            for (int k = 0; k < localFeat.rows; k++)
            {
                //vector<int> v(localFeat.cols);
                //localFeat.row(k).copyTo(v);

                // keep in mind that the coordinates of the rectangles are obeying Matlab indexing, therefore adaptation is performed explicitly here

                double a1 = localImg.at<double>(localFeat.at<uchar>(k, 3) - 1, localFeat.at<uchar>(k, 2) - 1) -
                            localImg.at<double>(localFeat.at<uchar>(k, 3) - 1, localFeat.at<uchar>(k, 0) - 2) -
                            localImg.at<double>(localFeat.at<uchar>(k, 1) - 2, localFeat.at<uchar>(k, 2) - 1) +
                            localImg.at<double>(localFeat.at<uchar>(k, 1) - 2, localFeat.at<uchar>(k, 0) - 2);
                double a2 = localImg.at<double>(localFeat.at<uchar>(k, 7) - 1, localFeat.at<uchar>(k, 6) - 1) -
                            localImg.at<double>(localFeat.at<uchar>(k, 7) - 1, localFeat.at<uchar>(k, 4) - 2) -
                            localImg.at<double>(localFeat.at<uchar>(k, 5) - 2, localFeat.at<uchar>(k, 6) - 1) +
                            localImg.at<double>(localFeat.at<uchar>(k, 5) - 2, localFeat.at<uchar>(k, 4) - 2);
                /*double a3 = localImg.at<double>(localFeat.at<uchar>(k, 11) - 1, localFeat.at<uchar>(k, 10) - 1) -
                    localImg.at<double>(localFeat.at<uchar>(k, 11) - 1, localFeat.at<uchar>(k, 8) - 2) -
                    localImg.at<double>(localFeat.at<uchar>(k, 9) - 2, localFeat.at<uchar>(k, 10) - 1) +
                    localImg.at<double>(localFeat.at<uchar>(k, 9) - 2, localFeat.at<uchar>(k, 8) - 2);
                double a4 = localImg.at<double>(localFeat.at<uchar>(k, 15) - 1, localFeat.at<uchar>(k, 14) - 1) -
                    localImg.at<double>(localFeat.at<uchar>(k, 15) - 1, localFeat.at<uchar>(k, 12) - 2) -
                    localImg.at<double>(localFeat.at<uchar>(k, 13) - 2, localFeat.at<uchar>(k, 14) - 1) +
                    localImg.at<double>(localFeat.at<uchar>(k, 13) - 2, localFeat.at<uchar>(k, 12) - 2);*/

                double z1 = localMask.at<double>(localFeat.at<uchar>(k, 3) - 1, localFeat.at<uchar>(k, 2) - 1) -
                            localMask.at<double>(localFeat.at<uchar>(k, 3) - 1, localFeat.at<uchar>(k, 0) - 2) -
                            localMask.at<double>(localFeat.at<uchar>(k, 1) - 2, localFeat.at<uchar>(k, 2) - 1) +
                            localMask.at<double>(localFeat.at<uchar>(k, 1) - 2, localFeat.at<uchar>(k, 0) - 2);
                double z2 = localMask.at<double>(localFeat.at<uchar>(k, 7) - 1, localFeat.at<uchar>(k, 6) - 1) -
                            localMask.at<double>(localFeat.at<uchar>(k, 7) - 1, localFeat.at<uchar>(k, 4) - 2) -
                            localMask.at<double>(localFeat.at<uchar>(k, 5) - 2, localFeat.at<uchar>(k, 6) - 1) +
                            localMask.at<double>(localFeat.at<uchar>(k, 5) - 2, localFeat.at<uchar>(k, 4) - 2);
                /*double z3 = localMask.at<double>(localFeat.at<uchar>(k, 11) - 1, localFeat.at<uchar>(k, 10) - 1) -
                    localMask.at<double>(localFeat.at<uchar>(k, 11) - 1, localFeat.at<uchar>(k, 8) - 2) -
                    localMask.at<double>(localFeat.at<uchar>(k, 9) - 2, localFeat.at<uchar>(k, 10) - 1) +
                    localMask.at<double>(localFeat.at<uchar>(k, 9) - 2, localFeat.at<uchar>(k, 8) - 2);
                double z4 = localMask.at<double>(localFeat.at<uchar>(k, 15) - 1, localFeat.at<uchar>(k, 14) - 1) -
                    localMask.at<double>(localFeat.at<uchar>(k, 15) - 1, localFeat.at<uchar>(k, 12) - 2) -
                    localMask.at<double>(localFeat.at<uchar>(k, 13) - 2, localFeat.at<uchar>(k, 14) - 1) +
                    localMask.at<double>(localFeat.at<uchar>(k, 13) - 2, localFeat.at<uchar>(k, 12) - 2);*/

                double mz1 = (z1 > 1) ? z1 : 1;
                double mz2 = (z2 > 1) ? z2 : 1;
                /*double mz3 = (z3 > 1) ? z3 : 1;
                double mz4 = (z4 > 1) ? z4 : 1;*/

                //double featVal = a1 / mz1 + a3 / mz3 - a2 / mz2 - a4 / mz4;
                double featVal = a1 / mz1 - a2 / mz2;
                feats.at<double>(0, i * localFeat.rows + k) = featVal;
            }
        }

        return feats;
    }

    vector<Mat> RV_channelsExtractionSingleFrame(Mat frame, paramList params)
    {
        vector<Mat> output(params.noChannels);
        for (int i = 0; i < params.noChannels; i++)
        {
            Mat buff(frame.rows, frame.cols, CV_64FC1);
            buff.setTo(0);
            output[i] = buff;
        }

        output[0] = frame;
        Mat dx(frame.rows, frame.cols, CV_64FC1);
        Mat dy(frame.rows, frame.cols, CV_64FC1);

        Sobel(frame, dx, CV_64FC1, 0, 1);
        Sobel(frame, dy, CV_64FC1, 1, 0);

        Mat magnitudeAux(frame.rows, frame.cols, CV_64FC1);
        Mat orientation(frame.rows, frame.cols, CV_64FC1);

        for (int i = 0; i < frame.rows; i++)
        {
            for (int j = 0; j < frame.cols; j++)
            {
                magnitudeAux.at<double>(i, j) = sqrt(pow(dx.at<double>(i, j), 2) + pow(dy.at<double>(i, j), 2));
                orientation.at<double>(i, j) = atan2(dx.at<double>(i, j), dy.at<double>(i, j));
            }
        }

        Mat magnitude(frame.rows, frame.cols, CV_64FC1);
        log(magnitudeAux + 1.0, magnitude); // switch to log scale to attenuate differences...

        output[1] = magnitude;

        /*namedWindow("frame", CV_WINDOW_AUTOSIZE);
        int flag = RV_imshow("frame", frame);
        namedWindow("magnitude", CV_WINDOW_AUTOSIZE);
        flag = RV_imshow("magnitude", magnitude);
        namedWindow("orientation", CV_WINDOW_AUTOSIZE);
        flag = RV_imshow("orientation", orientation);*/

        //setMouseCallback("image2", onMouse, 0);

        // compute gradient histograms for each of the 6 discrete orientation intervals
        // if an orient falls inside a particular interval, we assign the pixel a value proportional
        // to the gradient magnitude weighted by a peaked Gaussian whose mean is the center of the
        // orientation interval and std = 1/(sqrt(2*pi))/5 (5 comes from the attempt of replicating Dollars toolbox)
        // of course, pdf(mu) should be 1, so we compensate the small std by dividing the pdf by 5

        // using Sobel gradient results in orients in the range (-pi;pi) therefore we split this interval in 6 complementary subintervals

        double sigma = 1 / sqrt(2 * M_PI) / 5;
        for (int k = 0; k < 6; k++)
        {
            double mu = -5 * M_PI / 6 + k * 2 * M_PI / 6;
            Mat localChannel(frame.rows, frame.cols, CV_64FC1);
            for (int i = 0; i < frame.rows; i++)
            {
                for (int j = 0; j < frame.cols; j++)
                {
                    double valuePdf = RV_gaussPdf(orientation.at<double>(i, j), mu, sigma) / 5;
                    localChannel.at<double>(i, j) = magnitude.at<double>(i, j) * valuePdf;
                }
            }
            output[k + 2] = localChannel;
        }

        // compute the LBP map of the original (grayscaled) depth image
        Mat lbpMap = RV_lbp(frame);
        Mat lbpMap64(lbpMap.rows, lbpMap.cols, CV_64FC1);
        lbpMap.convertTo(lbpMap64, CV_64FC1);

        for (int i = 1; i < frame.rows - 1; i++)
        {
            for (int j = 1; j < frame.cols - 1; j++)
            {
                output[8].at<double>(i, j) = lbpMap64.at<double>(i - 1, j - 1);
            }
        }
        return output;
    }

    double RV_gaussPdf(double x, double mu, double sigma)
    {
        /*
            x = double(int(x * 10)) / 10;
            static map<double, double> pdfMap;
            if (pdfMap.find(x) != pdfMap.end())
            {
                return pdfMap[x];
            }*/
        double aux = -pow(x - mu, 2) / (2 * pow(sigma, 2));
        double out = 1 / (sqrt(2 * M_PI) * sigma) * exp(aux);
        /*pdfMap[x] = out;*/
        return out;

    }

    int RV_imshow(string windowName, Mat img)
    {
        double minVal, maxVal;
        Mat draw(img.rows, img.cols, CV_8UC1);
        minMaxLoc(img, &minVal, &maxVal);
        img.convertTo(draw, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        imshow(windowName, draw);
        return 1;
    }

    Mat RV_lbp(const Mat &src)
    {
        Mat dst;
        dst = Mat::zeros(src.rows - 2, src.cols - 2, CV_8UC1);
        for (int i = 1; i < src.rows - 1; i++) {
            for (int j = 1; j < src.cols - 1; j++) {
                uchar center = (uchar)src.at<double>(i, j);
                unsigned char code = 0;
                code |= ((uchar)src.at<double>(i - 1, j - 1) > center) << 7;
                code |= ((uchar)src.at<double>(i - 1, j) > center) << 6;
                code |= ((uchar)src.at<double>(i - 1, j + 1) > center) << 5;
                code |= ((uchar)src.at<double>(i, j + 1) > center) << 4;
                code |= ((uchar)src.at<double>(i + 1, j + 1) > center) << 3;
                code |= ((uchar)src.at<double>(i + 1, j) > center) << 2;
                code |= ((uchar)src.at<double>(i + 1, j - 1) > center) << 1;
                code |= ((uchar)src.at<double>(i, j - 1) > center) << 0;
                dst.at<uchar>(i - 1, j - 1) = code;
            }
        }
        return 255 - dst;
    }

}