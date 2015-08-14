#pragma once

#ifndef KINECTSDKGRABBER_H
#define KINECTSDKGRABBER_H

#include <Grabber/GrabberBase.h>
#include <DataObject/IDataObject.h>

#include <Windows.h>
#include <NuiApi.h>

namespace hpe
{
    class KinectCoordinatesMapper : public IDataObject
    {
        public:
            KinectCoordinatesMapper() : correspondencies(nullptr) {}
            KinectCoordinatesMapper(LONG *c) : correspondencies(c) {}
            LONG *correspondencies;
    };

    class KinectSDKGrabber : public GrabberBase
    {
        public:
            KinectSDKGrabber(void);
            ~KinectSDKGrabber(void);

            INuiCoordinateMapper *GetCoordinateMapper();

        protected:
            bool GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame) override;
            IDataStorage::Ptr CreateDataStorage() override;
            void PreRun() override;

        private:
            static const int FrameWidth = 640;
            static const int FrameHeight = 480;
            static const int FrameBufferSize = FrameWidth *FrameHeight;

            HRESULT CreateFirstConnected();
            HRESULT ProcessDepth(float *buffer);
            HRESULT ProcessColor(cv::Mat &mat);

            INuiSensor *m_pNuiSensor;
            HANDLE m_pDepthStreamHandle;
            HANDLE m_hNextDepthFrameEvent;
            HANDLE m_pColorStreamHandle;
            HANDLE m_hNextColorFrameEvent;

            LONG *m_colorCoordinates;
    };

}

#endif