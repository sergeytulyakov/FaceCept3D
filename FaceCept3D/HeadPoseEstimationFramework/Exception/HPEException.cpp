#include "HPEException.h"

HPEException::HPEException(std::string message)
    : m_message(message)
{

}

const char *HPEException::what()
{
    return m_message.c_str();
}
