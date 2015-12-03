#pragma once

#ifndef FACIALEXPRESSIONSPROCESSOR_H
#define FACIALEXPRESSIONSPROCESSOR_H

#include <Processor/IProcessor.h>
#include <Features/Sampler/CylinderSampler.h>
#include <Features/Calculator/CylinderOptimizedFeatureCalculator.h>
#include <FacialExpressions/ferLocalFunctions.h>
#include <pcl/point_types.h>
#include <boost/signals2/signal.hpp>

namespace hpe
{
    class FacialExpressionProcessor: public IProcessor
    {
        public:
            typedef boost::signals2::signal<void (std::vector<double>)> FacialExressionReadySignal;

            FacialExpressionProcessor();
            FacialExpressionProcessor(std::string dataFolder);
            void Process(IDataStorage::Ptr dataStorage);
            void Init();
            void SubscribeFacialExpressionReadySignal(const FacialExressionReadySignal::slot_type &slot)
            {
                m_facialExpressionReady.connect(slot);
            }
        private:
            typedef pcl::PointXYZRGBNormal PointT;
            typedef pcl::PointXYZRGBNormal NormalT;

            void Init(std::string);
            std::shared_ptr<CylinderSampler> CreateCylindricalSampler();
            void ThreadRoutine();

            fer::paramList m_ferParameters;
            std::vector<std::vector<fer::randomTree>> m_forests;
            std::shared_ptr<hpe::CylinderOptimizedFeatureCalculator<PointT, NormalT>> m_featureCalculator;

            bool m_workerBusy;
            bool m_dataReady;
            std::vector<double> m_ferResult;
            Common<PointT>::Landmarks m_landmarks;
            pcl::PointCloud<PointT> m_cloud;
            int m_numberOfClasses;

            FacialExressionReadySignal m_facialExpressionReady;
    };
}

#endif