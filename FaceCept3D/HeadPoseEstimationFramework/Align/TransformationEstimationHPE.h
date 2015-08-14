#ifndef TRANSFORMATIONESTIMATION_H
#define TRANSFORMATIONESTIMATION_H

#include <pcl/point_cloud.h>

#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>

namespace hpe
{
	/**
	 \class	TransformationEstimationHPE
	
	 \brief	Class that performs history-based weighting of the points for ICP
			Thanks to this, ICP is speeded up 4 times. The method overloads 
			the standard pcl class to get the desired functionality

	
	 \tparam	PointSource	PointType of the source point cloud.
	 \tparam	PointTarget	PointType of the target point cloud.
	 \tparam	MatScalar  	scalar type.
	 */

	template <typename PointSource, typename PointTarget, typename MatScalar = float>
    class TransformationEstimationHPE : public pcl::registration::TransformationEstimationPointToPlaneWeighted<PointSource, PointTarget, MatScalar>
    {
            typedef TransformationEstimationHPE<PointSource, PointTarget, MatScalar> Self;
            typedef pcl::registration::TransformationEstimationPointToPlaneWeighted<PointSource, PointTarget, MatScalar> Base;
            typedef typename Base::Matrix4 Matrix4;
        public:
            typedef boost::shared_ptr<TransformationEstimationHPE<PointSource, PointTarget, MatScalar>> Ptr;

            TransformationEstimationHPE()
            {
            }

            void SetWeights(std::vector<double> &weights)
            {
                m_weights = weights;
            }

            inline void
            estimateRigidTransformation(
                const pcl::PointCloud<PointSource> &cloud_src,
                const pcl::PointCloud<PointTarget> &cloud_tgt,
                Matrix4 &transformation_matrix) const
            {
                Base::estimateRigidTransformation(cloud_src, cloud_tgt, transformation_matrix);
            }

            inline void
            estimateRigidTransformation(
                const pcl::PointCloud<PointSource> &cloud_src,
                const std::vector<int> &indices_src,
                const pcl::PointCloud<PointTarget> &cloud_tgt,
                Matrix4 &transformation_matrix) const
            {
                Base::estimateRigidTransformation(cloud_src, indices_src, cloud_tgt, transformation_matrix);
            }

            void
            estimateRigidTransformation(
                const pcl::PointCloud<PointSource> &cloud_src,
                const std::vector<int> &indices_src,
                const pcl::PointCloud<PointTarget> &cloud_tgt,
                const std::vector<int> &indices_tgt,
                Matrix4 &transformation_matrix) const
            {
                std::vector<double> weights(indices_src.size());

                if (m_weights.size() != 0)
                {
                    for (int i = 0; i < indices_src.size(); i += 1)
                    {
                        weights[i] = m_weights[indices_src[i]];
                    }
                }
                else
                {
                    for (int i = 0; i < indices_src.size(); i += 1)
                    {
                        weights[i] = 1;
                    }
                }

                Self *self = const_cast<Self *>(this);
                self->Base::setWeights(weights);
                self->Base::setUseCorrespondenceWeights(false);

                Base::estimateRigidTransformation(cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);
            }

            void
            estimateRigidTransformation(
                const pcl::PointCloud<PointSource> &cloud_src,
                const pcl::PointCloud<PointTarget> &cloud_tgt,
                const pcl::Correspondences &correspondences,
                Matrix4 &transformation_matrix) const
            {
                Base::estimateRigidTransformation(cloud_src, cloud_tgt, correspondences, transformation_matrix);
            }

        private:
            std::vector<double> m_weights;
    };
}

#endif // TRANSFORMATIONESTIMATION_H
