#pragma once

#ifndef DEPTHPREPROCESSINGPROCESSOR_H
#define DEPTHPREPROCESSINGPROCESSOR_H

#include <Processor/IProcessor.h>
#include <Converter/IDataConverter.h>

#include <opencv2/opencv.hpp>


namespace hpe
{
    /**
     \class	DepthPreprocessingProcessor
    
     \brief	Depth smoothing, closing holes are done here. 
			To get an idea, have a look at the Process code.
    
     \author	Sergey
     \date	8/11/2015
     */

    class DepthPreprocessingProcessor : public IProcessor
    {
        public:
            typedef std::shared_ptr<IDataConverter> ConverterPtr;

            DepthPreprocessingProcessor(void);
            DepthPreprocessingProcessor(int erodeSize, int closingSize, int gaussianSize, float gaussianSigma, float threshold);
            ~DepthPreprocessingProcessor(void);

            void Process(IDataStorage::Ptr storage) override;
            void SetConverter(ConverterPtr c);

        private:
            cv::Mat AndMask(cv::Mat &matrix, cv::Mat &mat);

            std::string m_framesKey;

            int m_gaussianSize;
            int m_erodeSize;
            int m_closingSize;
            float m_gaussianSigma;
            float m_distanceThreshold;

            bool m_saveOriginalCloud;
            ConverterPtr m_converter;
    };

}

#endif