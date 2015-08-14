#pragma once

#ifndef GRABBERBASE_H
#define GRABBERBASE_H

#include <opencv2/opencv.hpp>

#include <DataObject/IDataObject.h>
#include <DataObject/IDataStorage.h>
#include <Processor/IProcessor.h>

#include <vector>

namespace hpe
{
    /**
     \class	GrabberBase
    
     \brief	A generic component to decouple technical sensor details from the 
			recognition framework. Typically it should be subclassed for any 
			sensor.

			Any grabber takes a sequence of processors to that perform a desired 
			functionality. Basically, recognition methods do not know anything about 
			processors and grabbers, they just do recognition. Grabber doesn't know 
			anything about recognition methods, but knows how to call processors,
			which in turn do not know anything about technical details of the 
			sensor, but rather accept the data from the grabber, process it,
			and pass forward.
    
     \author	Sergey
     \date	8/11/2015
     */

    class GrabberBase
    {
        public:
            GrabberBase();
            void Start();
            void Stop();
            void AddProcessor(IProcessor::Ptr processor);

        protected:
            virtual bool GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame) = 0;
            virtual IDataStorage::Ptr CreateDataStorage() = 0;
            virtual void PreRun() = 0;

        private:
            bool m_run;
            std::vector<IProcessor::Ptr> m_processors;
            std::vector<IProcessor::Ptr> m_processorsToAdd;
    };

}

#endif