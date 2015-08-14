#include "KinectSDKGrabber.h"

#include <DataObject/MapDataStorage.h>
#include <Exception/HPEException.h>

#include <pcl/common/time.h>

namespace hpe
{

    KinectSDKGrabber::KinectSDKGrabber(void)
    {
        m_colorCoordinates = new LONG[FrameBufferSize * 2];
    }

    KinectSDKGrabber::~KinectSDKGrabber(void)
    {
        delete [] m_colorCoordinates;
    }

    IDataStorage::Ptr KinectSDKGrabber::CreateDataStorage()
    {
        MapDataStorage::Ptr storage(new MapDataStorage);
        IDataObject::Ptr mapper(new KinectCoordinatesMapper(m_colorCoordinates));
        storage->Set("CoordinatesMapper", mapper);
        return storage;
    }

    void KinectSDKGrabber::PreRun()
    {
        HRESULT hr = CreateFirstConnected();
        if (FAILED(hr))
        {
            throw HPEException("KinectSDKGrabber::PreRun() - CreateFirstConnected() failed");
        }
    }

    bool KinectSDKGrabber::GetNextFrame(cv::Mat &colorFrame, cv::Mat &depthFrame)
    {
        WaitForSingleObject(m_hNextDepthFrameEvent, INFINITE);
        WaitForSingleObject(m_hNextColorFrameEvent, INFINITE);

        colorFrame = cv::Mat::zeros(FrameHeight, FrameWidth, CV_8UC4);
        depthFrame = cv::Mat::zeros(FrameHeight, FrameWidth, CV_32FC1);

        HRESULT hr = ProcessDepth((float *)depthFrame.data);
        if (FAILED(hr))
        {
            return false;
        }

        hr = ProcessColor(colorFrame);
        if (FAILED(hr))
        {
            return false;
        }

        return true;
    }

    HRESULT KinectSDKGrabber::CreateFirstConnected()
    {
        INuiSensor *pNuiSensor;
        HRESULT hr;

        int iSensorCount = 0;
        hr = NuiGetSensorCount(&iSensorCount);
        if (FAILED(hr))
        {
            return hr;
        }
        std::cout << "Count" << std::endl;

        // Look at each Kinect sensor
        for (int i = 0; i < iSensorCount; ++i)
        {
            // Create the sensor so we can check status, if we can't create it, move on to the next
            hr = NuiCreateSensorByIndex(i, &pNuiSensor);
            if (FAILED(hr))
            {
                continue;
            }
            std::cout << "Created" << std::endl;

            // Get the status of the sensor, and if connected, then we can initialize it
            hr = pNuiSensor->NuiStatus();
            if (S_OK == hr)
            {
                m_pNuiSensor = pNuiSensor;
                break;
            }

            // This sensor wasn't OK, so release it since we're not using it
            pNuiSensor->Release();
        }

        if (NULL != m_pNuiSensor)
        {
            // Initialize the Kinect and specify that we'll be using depth
            hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
            if (SUCCEEDED(hr))
            {
                // Create an event that will be signaled when depth data is available
                m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

                // Open a depth image stream to receive depth frames
                hr = m_pNuiSensor->NuiImageStreamOpen(
                         NUI_IMAGE_TYPE_DEPTH,
                         NUI_IMAGE_RESOLUTION_640x480,
                         0,
                         2,
                         m_hNextDepthFrameEvent,
                         &m_pDepthStreamHandle);

                m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);

                std::cout << "Depth" << std::endl;

            }

            if (SUCCEEDED(hr))
            {
                m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

                hr = m_pNuiSensor->NuiImageStreamOpen(
                         NUI_IMAGE_TYPE_COLOR,
                         NUI_IMAGE_RESOLUTION_640x480,
                         0,
                         2,
                         m_hNextColorFrameEvent,
                         &m_pColorStreamHandle);
                std::cout << "Color" << std::endl;
            }
        }

        if (NULL == m_pNuiSensor || FAILED(hr))
        {
            return E_FAIL;
        }

        return hr;
    }

    HRESULT KinectSDKGrabber::ProcessDepth(float *buffer)
    {
        HRESULT hr = S_OK;
        NUI_IMAGE_FRAME imageFrame;

        // Attempt to get the depth frame
        hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
        HRESULT nodata = E_NUI_FRAME_NO_DATA;
        if (FAILED(hr) && hr != E_NUI_FRAME_NO_DATA)
        {
            std::cout << hr << std::endl;
            return hr;
        }

        BOOL nearMode;
        INuiFrameTexture *pTexture = imageFrame.pFrameTexture;
        NUI_LOCKED_RECT LockedRect;

        // Lock the frame data so the Kinect knows not to modify it while we're reading it
        pTexture->LockRect(0, &LockedRect, NULL, 0);

        m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
            NUI_IMAGE_RESOLUTION_640x480,
            NUI_IMAGE_RESOLUTION_640x480,
            FrameBufferSize,
            (USHORT *)LockedRect.pBits,
            FrameBufferSize * 2,
            m_colorCoordinates
        );

        pTexture->UnlockRect(0);

        // Get the depth image pixel texture
        hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
        if (FAILED(hr))
        {
            goto ReleaseFrame;
        }

        // Lock the frame data so the Kinect knows not to modify it while we're reading it
        pTexture->LockRect(0, &LockedRect, NULL, 0);

        // Make sure we've received valid data
        if (LockedRect.Pitch != 0)
        {
            const NUI_DEPTH_IMAGE_PIXEL *pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

            // end pixel is start + width*height - 1
            const NUI_DEPTH_IMAGE_PIXEL *pBufferEnd = pBufferRun + FrameBufferSize;


            int idx = 0;
            while (pBufferRun < pBufferEnd)
            {
                float depth = pBufferRun->depth;
                if (depth != 0)
                {
                    buffer[idx] = depth / 1000.f;
                }
                else
                {
                    buffer[idx] = std::numeric_limits<float>::quiet_NaN();
                }
                ++pBufferRun;
                ++idx;
            }

            hr = S_OK;
        }
        else
        {
            hr = E_FAIL;
        }

        // We're done with the texture so unlock it
        pTexture->UnlockRect(0);

        pTexture->Release();

    ReleaseFrame:
        // Release the frame
        m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

        return hr;
    }

    HRESULT KinectSDKGrabber::ProcessColor(cv::Mat &mat)
    {
        NUI_IMAGE_FRAME imageFrame;

        HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
        if (FAILED(hr))
        {
            return hr;
        }

        NUI_LOCKED_RECT LockedRect;
        hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
        if (FAILED(hr))
        {
            goto ReleaseFrame;
        }

        if (LockedRect.Pitch != 0)
        {
            if (0)
            {
                int idx = 0;
                while (idx < FrameBufferSize)
                {
                    *(mat.data + 3 * idx + 2) = *(LockedRect.pBits + 4 * idx + 2);
                    *(mat.data + 3 * idx + 1) = *(LockedRect.pBits + 4 * idx + 1);
                    *(mat.data + 3 * idx + 0) = *(LockedRect.pBits + 4 * idx + 0);
                    ++idx;
                }
            }
            else
            {
                memcpy(mat.data, LockedRect.pBits, LockedRect.size);
            }

            hr = S_OK;
        }
        else
        {
            hr = E_FAIL;
        }


        imageFrame.pFrameTexture->UnlockRect(0);

    ReleaseFrame:
        hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

        return hr;

    }

    INuiCoordinateMapper *KinectSDKGrabber::GetCoordinateMapper()
    {
        if (m_pNuiSensor)
        {
            INuiCoordinateMapper *mapper;
            HRESULT hr = m_pNuiSensor->NuiGetCoordinateMapper(&mapper);
            if (SUCCEEDED(hr))
            {
                return mapper;
            }
            else
            {
                return NULL;
            }
        }
        return NULL;
    }

}