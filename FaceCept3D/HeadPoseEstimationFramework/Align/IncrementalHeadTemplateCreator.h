#pragma once

#ifndef INCREMENTALHEADTEMPLATECREATOR_H
#define INCREMENTALHEADTEMPLATECREATOR_H

#include <Align/IHeadTemplateCreator.h>
#include <Align/CloudMapper.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hpe
{
    /**
     \class	IncrementalHeadTemplateCreator
    
     \brief	Class that performs incremental alighment. 
    
     \author	Sergey
     \date	8/11/2015
     */

    class IncrementalHeadTemplateCreator : public IHeadTemplateCreator<pcl::PointXYZRGBA>
    {
        public:
            typedef pcl::PointXYZRGBA PointType;
            typedef pcl::PointCloud<PointType> CloudType;

            IncrementalHeadTemplateCreator(void);
            ~IncrementalHeadTemplateCreator(void);

            CloudType::Ptr GetTemplate();

            CloudType::Ptr AddCloudToTemplate(CloudType::Ptr cloud);

        private:
            CloudType::Ptr m_template;
            CloudType::Ptr m_targetCloud;
            CloudMapper m_mapper;
            CloudMapper::DynamicICPParams m_icpParams;
            Eigen::Matrix4f m_aggregatedTransform;
    };

}

#endif