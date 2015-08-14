#pragma once
#ifndef OPENNIGRABBER_H
#define OPENNIGRABBER_H

#include <Grabber/GrabberBase.h>

#include <pcl/io/openni_grabber.h>
#include <DataObject/IDataStorage.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

namespace hpe
{

    class OpenNIGrabber : public GrabberBase
    {
        public:
            OpenNIGrabber(void);
            ~OpenNIGrabber(void);

        protected:
            void PreRun();
            hpe::IDataStorage::Ptr CreateDataStorage();
            bool GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame);

        private:
            cv::Mat GetColorFrame(const boost::shared_ptr<openni_wrapper::Image> &img);
            cv::Mat GetDepthFrame(const boost::shared_ptr<openni_wrapper::DepthImage> &depthImg);

            void GrabberCallback(const boost::shared_ptr<openni_wrapper::Image> &color, const boost::shared_ptr<openni_wrapper::DepthImage> &depth, float c);

            std::shared_ptr<pcl::OpenNIGrabber> m_openniGrabber;
            cv::Mat m_colorFrame;
            cv::Mat m_depthFrame;
            bool m_haveFrames;
    };

}

#endif
