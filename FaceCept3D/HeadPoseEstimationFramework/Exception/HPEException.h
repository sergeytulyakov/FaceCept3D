#ifndef HPEEXCEPTION_H
#define HPEEXCEPTION_H

#include <stdexcept>

class HPEException : public std::exception
{
    public:
        HPEException(std::string message);
        virtual const char *what();

    private:
        std::string m_message;
};

#endif // HPEEXCEPTION_H
