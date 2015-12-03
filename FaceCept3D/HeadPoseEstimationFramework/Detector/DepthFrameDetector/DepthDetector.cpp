#include "DepthDetector.h"

#include <boost/math/special_functions/fpclassify.hpp>

namespace hpe
{

    DepthDetector::DepthDetector(std::string dataFolder)
    {
        Init(dataFolder);
    }

    DepthDetector::~DepthDetector(void)
    {
    }

    hpe::headPoseInfo DepthDetector::Detect(cv::Mat depthFrame)
    {
        cv::Mat converted(depthFrame.rows, depthFrame.cols, depthFrame.type());

        for (int y = 0; y < depthFrame.rows; ++y)
        {
            for (int x = 0; x < depthFrame.cols; ++x)
            {
                float depthValue = depthFrame.at<float>(y, x);

                if (depthValue > 1.5 || depthValue == 0.0f || boost::math::isnan(depthValue))
                {
                    depthValue = 0;
                }
                else
                {
                    depthValue = depthValue * 1000;
                }

                converted.at<float>(y, x) = depthValue;
            }
        }

        headPoseInfo info = estimateHeadAngles(converted, m_cascade, m_paramList);
        return info;
    }

    void DepthDetector::Init(std::string dataFolder)
    {
        std::string configFileName = dataFolder + "/config.txt";
        loadConfig(configFileName, &m_paramList);

        std::string modelsDir = dataFolder + "/models/";
        for (int i = 0; i < 5; i++)
        {
            m_cascade.push_back(readTree(modelsDir, i + 1));
        }
    }

}