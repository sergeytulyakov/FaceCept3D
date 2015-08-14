#ifndef HEADPOSEINFO_H
#define HEADPOSEINFO_H

#include <DataObject/IDataObject.h>

namespace hpe
{
    class HeadPoseInformation : public IDataObject
    {
        public:
            typedef std::shared_ptr<HeadPoseInformation> Ptr;

            HeadPoseInformation()
            {
                Yaw = 0;
                Tilt = 0;
                Roll = 0;
                WorldX = 0;
                WorldY = 0;
                WorldZ = 0;
                DepthFrameX = 0;
                DepthFrameY = 0;
                HasHeadInfo = false;
            }

            double Yaw;
            double Tilt;
            double Roll;
            double WorldX;
            double WorldY;
            double WorldZ;
            double DepthFrameX;
            double DepthFrameY;
            bool HasHeadInfo;
    };
}

#endif