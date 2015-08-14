#include "IncrementalHeadTemplateCreator.h"
#include <Filter/Filters.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <UI/ShowCloud.h>

namespace hpe
{

    IncrementalHeadTemplateCreator::IncrementalHeadTemplateCreator(void)
        : m_template(new CloudType), m_targetCloud(new CloudType)
    {
        m_mapper.SetUseNormalShooting(false);
        m_mapper.SetUsePointToPlaneMetric(true);

        m_icpParams.push_back(CloudMapper::ICPParams(0.03, 0.005, 10e-6, 400, 10));
        m_mapper.SetDynamicICPParams(m_icpParams);

        m_aggregatedTransform = Eigen::Matrix4f::Identity();
    }


    IncrementalHeadTemplateCreator::~IncrementalHeadTemplateCreator(void)
    {
    }

    IncrementalHeadTemplateCreator::CloudType::Ptr IncrementalHeadTemplateCreator::GetTemplate()
    {
        CloudType::Ptr result(new CloudType);
        pcl::copyPointCloud(*m_template, *result);
        return result;
    }

    IncrementalHeadTemplateCreator::CloudType::Ptr IncrementalHeadTemplateCreator::AddCloudToTemplate(CloudType::Ptr cloud)
    {
        if (m_template->points.size() == 0)
        {
            pcl::copyPointCloud(*cloud, *m_template);
            pcl::copyPointCloud(*cloud, *m_targetCloud);
        }
        else
        {
            pcl::ScopeTime t("AddCloudToTemplate");
            Eigen::Matrix4f transformMatrix = m_mapper.GetTransformForTwoCloudsDynamically(cloud, m_targetCloud, m_icpParams);
            m_aggregatedTransform *= transformMatrix;

            CloudType::Ptr transformedSource(new CloudType);

            pcl::transformPointCloud(*cloud, *transformedSource, m_aggregatedTransform);

            *m_template += *transformedSource;
            m_template = Voxelize<PointType>(m_template, 0.001f);

            pcl::copyPointCloud(*cloud, *m_targetCloud);
        }

        return GetTemplate();
    }

}