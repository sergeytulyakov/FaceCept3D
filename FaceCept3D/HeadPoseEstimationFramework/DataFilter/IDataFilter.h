#ifndef IDATAFILTER_H
#define IDATAFILTER_H

#include "Interface/IInterface.h"

namespace hpe
{
    template <typename DataType>
    class DataFilter : public IInterface
    {
        public:
            virtual DataType Filter(DataType &input) = 0;
    };
}

#endif // IDATAFILTER_H
