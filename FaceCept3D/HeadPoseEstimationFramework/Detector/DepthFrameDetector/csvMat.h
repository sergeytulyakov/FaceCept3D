#pragma once
#ifndef CSVMAT_H
#define CSVMAT_H

#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdint.h>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

namespace hpe
{

    Mat readCSVMat(string csvFrameName, int lin, int col);

}

#endif