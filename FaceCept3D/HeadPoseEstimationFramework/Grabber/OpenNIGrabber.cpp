#include "OpenNIGrabber.h"

#include <DataObject/MapDataStorage.h>

namespace hpe
{

    OpenNIGrabber::OpenNIGrabber(void)
    {
    }

    OpenNIGrabber::~OpenNIGrabber(void)
    {
    }

    void OpenNIGrabber::PreRun()
    {
        m_openniGrabber = std::shared_ptr<pcl::OpenNIGrabber>(new pcl::OpenNIGrabber());
        m_haveFrames = false;

        boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float)> f =
            boost::bind(&OpenNIGrabber::GrabberCallback, this, _1, _2, _3);

        m_openniGrabber->registerCallback(f);
        m_openniGrabber->start();
    }

    hpe::IDataStorage::Ptr OpenNIGrabber::CreateDataStorage()
    {
        return IDataStorage::Ptr(new MapDataStorage);
    }

    cv::Mat OpenNIGrabber::GetColorFrame(const boost::shared_ptr<openni_wrapper::Image> &img)
    {
        cv::Mat frameRGB = cv::Mat(img->getHeight(), img->getWidth(), CV_8UC3);

        img->fillRGB(frameRGB.cols, frameRGB.rows, frameRGB.data, frameRGB.step);
        cv::Mat frameBGR;
        cv::cvtColor(frameRGB, frameBGR, cv::COLOR_RGB2BGR);
        return frameBGR;
    }

    cv::Mat OpenNIGrabber::GetDepthFrame(const boost::shared_ptr<openni_wrapper::DepthImage> &depthImg)
    {
        cv::Mat frameDepth = cv::Mat(depthImg->getHeight(), depthImg->getWidth(), CV_32FC1);
        depthImg->fillDepthImage(frameDepth.cols, frameDepth.rows, (float *)frameDepth.data, frameDepth.step);
        return frameDepth;
    }

    void OpenNIGrabber::GrabberCallback(const boost::shared_ptr<openni_wrapper::Image> &color, const boost::shared_ptr<openni_wrapper::DepthImage> &depth, float c)
    {
        (void)c;

        cv::Mat colorMat = GetColorFrame(color);
        cv::Mat depthMat = GetDepthFrame(depth);

        colorMat.copyTo(m_colorFrame);
        depthMat.copyTo(m_depthFrame);

        m_haveFrames = true;
    }

    bool OpenNIGrabber::GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame)
    {
        while (m_haveFrames == false)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }
        m_haveFrames = false;

        m_colorFrame.copyTo(colorFrame);
        m_depthFrame.copyTo(depthFrame);

        return true;
    }

}
