#pragma once

#ifndef RAWFRAMES_H
#define RAWFRAMES_H

#include <DataObject/IDataObject.h>

#include <opencv2/opencv.hpp>

namespace hpe
{
    class RawFrames : public IDataObject
    {
        public:
            typedef std::shared_ptr<RawFrames> Ptr;

            cv::Mat colorFrame;
            cv::Mat depthFrame;
    };
}

#endif