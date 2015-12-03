#pragma once

#ifndef DEPTHDETECTOR_H
#define DEPTHDETECTOR_H

#include <opencv2/opencv.hpp>

#include "localFunctions.h"
#include "Tree.h"

namespace hpe
{
    class DepthDetector
    {
        public:
            DepthDetector(std::string dataFolder);
            ~DepthDetector(void);

            headPoseInfo Detect(cv::Mat depthFrame);
        private:
            void Init(std::string dataFolder);

            paramList m_paramList;
            std::vector<Tree> m_cascade;
    };
}

#endif
