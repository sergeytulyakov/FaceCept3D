#include "HeadExtractionProcessor.h"

#include <UI/PointPicker.h>

#include <DataObject/CloudXYZRGBA.h>
#include <DataObject/HeadPoseInfo.h>

#include <Filter/BoxFilter.h>
#include <Filter/Filters.h>
#include <Common.h>

namespace hpe
{

    HeadExtractionProcessor::HeadExtractionProcessor(void)
        : m_cloudKey("Cloud"), m_filteredCloudKey("FilteredCloud"), m_readyToExtract(false)
    {
    }

    HeadExtractionProcessor::~HeadExtractionProcessor(void)
    {
    }

    void HeadExtractionProcessor::Process(IDataStorage::Ptr storage)
    {
        CloudXYZRGBA::Ptr cloudObject = storage->GetAndCastNotNull<CloudXYZRGBA>(m_cloudKey);
        HeadPoseInformation::Ptr headPoseInfo = storage->GetAndCast<HeadPoseInformation>("HeadPoseInfo");

        if (m_readyToExtract == false)
        {
            if (headPoseInfo.get() != nullptr)
            {
                std::shared_ptr<ICloudFilter<TPoint>> boxFilter(new BoxFilter<TPoint>(
                                                       headPoseInfo->WorldX + 0.075f, headPoseInfo->WorldX - 0.075f,
                                                       headPoseInfo->WorldY + 0.1f, headPoseInfo->WorldY - 0.106f,
                                                       -1e6, 1e6));

                m_filteringQueue.AddFilter(boxFilter);
            }
            else
            {
                Common<pcl::PointXYZRGBA>::Landmarks landmarks;
                PointPicker<pcl::PointXYZRGBA> picker;
                picker.SetCloud(cloudObject->cloud);
                landmarks = picker.Pick("Selecte face boundaries", 4, "Select the four face boundaries in the following order:\n"
                                        "\t1. Leftmost point\n."
                                        "\t2. Rightmost point\n."
                                        "\t3. Topmost point\n."
                                        "\t4. Bottommost point (chin)\n"
                                        "NOTE: Make sure you see the front part of the face. Try rotating the view.");

                float delta = 0.03f;
                std::shared_ptr<ICloudFilter<TPoint>> boxFilter(new BoxFilter<TPoint>(
                                                       landmarks[0].point.x + delta, landmarks[1].point.x - delta,
                                                       landmarks[2].point.y + delta, landmarks[3].point.y - delta,
                                                       -1e6, 1e6));

                m_filteringQueue.AddFilter(boxFilter);
            }

            m_filteringQueue.AddFilterFunctor([](TCloud::Ptr & cloud) -> TCloud::Ptr
            {
                return Voxelize<TPoint>(cloud, 0.004f);
            });

            m_readyToExtract = true;
        }

        CloudXYZRGBA::Ptr filteredCloudObject(new CloudXYZRGBA);
        filteredCloudObject->cloud = m_filteringQueue.Filter(cloudObject->cloud);

        storage->Set(m_filteredCloudKey, filteredCloudObject);
    }

}
