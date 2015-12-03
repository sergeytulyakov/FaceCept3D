#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "csvMat.h"

using namespace cv;
using namespace std;
using std::vector;

#include <fstream>

namespace hpe
{

    Mat readCSVMat(string csvFrameName, int lin, int col)
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

}