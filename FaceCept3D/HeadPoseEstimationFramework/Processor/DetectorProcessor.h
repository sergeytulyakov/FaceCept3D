#pragma once

#ifndef DETECTORPROCESSOR_H
#define DETECTORPROCESSOR_H

#include <Processor/IProcessor.h>
#include <Detector/DepthFrameDetector/DepthDetector.h>

namespace hpe
{

    class DetectorProcessor : public hpe::IProcessor
    {
        public:
            DetectorProcessor(void);
            DetectorProcessor(std::string dataDir);
            ~DetectorProcessor(void);

            void Process(hpe::IDataStorage::Ptr dataStorage);

        private:
            hpe::DepthDetector m_detector;
    };

}

#endif
