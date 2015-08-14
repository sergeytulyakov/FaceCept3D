#pragma once

#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <list>

namespace hpe
{
    template <typename PointType>
    class Common
    {
        public:
            struct Landmark
            {
                Landmark() {}

                Landmark(int i, PointType p)
                    : index(i), point(p)
                {}

                int index;
                PointType point;
            };

            typedef std::vector<Landmark> Landmarks;
    };

    typedef std::vector<int> PointIndexes;
    typedef std::list<PointIndexes> PointHistory;
}

#endif // COMMON_H
