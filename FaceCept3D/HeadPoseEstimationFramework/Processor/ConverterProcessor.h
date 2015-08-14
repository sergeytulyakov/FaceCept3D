#pragma once

#ifndef CONVERTERPROCESSOR_H
#define CONVERTERPROCESSOR_H

#include <Processor/IProcessor.h>
#include <Converter/IDataConverter.h>

namespace hpe
{
    /**
     \class	ConverterProcessor
    
     \brief	Processor that converts RGB-D pair into point cloud.
			The converter should be provided by the client.

			In general converter contains sensor-specific information, 
			while converter processor is generic.

			The Process(...) method takes input data by using m_rawFramesKey,
			and puts the results to m_cloudKey
    
     \author	Sergey
     \date	8/11/2015
     */

    class ConverterProcessor : public IProcessor
    {
        public:
            typedef std::shared_ptr<IDataConverter> ConverterPtr;

            ConverterProcessor(void);
            ConverterProcessor(ConverterPtr converter);
            ~ConverterProcessor(void);

            void Init();
            void Process(IDataStorage::Ptr dataDtorage);

            void SetCloudKey(std::string key);
            void SetRawFramesKey(std::string key);
            void SetConverter(ConverterPtr converter);

        private:
            std::string m_cloudKey;
            std::string m_rawFramesKey;
            ConverterPtr m_converter;

            cv::Mat AndMask(cv::Mat &matrix, cv::Mat &mat);
    };

}

#endif