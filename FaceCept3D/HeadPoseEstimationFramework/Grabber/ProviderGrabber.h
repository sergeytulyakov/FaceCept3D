#pragma once

#ifndef PROVIDERGRABBER_H
#define PROVIDERGRABBER_H

#include <Grabber/GrabberBase.h>
#include <Provider/IDataProvider.h>
#include <pcl/point_types.h>

namespace hpe
{
    /**
     \class	ProviderGrabber
    
     \brief	A grabber that imitates usual grabber functionality,
			but read the frames from the the Provider, but not a 
			sensor.
    
     \author	Sergey
     \date	8/11/2015
     */

    class ProviderGrabber : public GrabberBase
    {
        public:
            typedef IDataProvider<pcl::PointXYZRGBA> ProviderType;

            ProviderGrabber(void);
            ~ProviderGrabber(void);

            void SetProvider(std::shared_ptr<ProviderType> provider);

        protected:
            bool GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame);
            IDataStorage::Ptr CreateDataStorage();
            void PreRun();

        private:
            std::shared_ptr<ProviderType> m_provider;
            int m_currentFrame;
    };

}

#endif