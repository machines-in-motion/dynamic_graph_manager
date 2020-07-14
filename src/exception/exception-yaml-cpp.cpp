/**
 * @file exception-yaml-cpp.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <stdarg.h>
#include <cstdio>
#include <dynamic_graph_manager/exception/exception-yaml-cpp.hpp>

using namespace dynamic_graph_manager;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionYamlCpp::EXCEPTION_NAME = "Tools";

ExceptionYamlCpp::ExceptionYamlCpp(
    const ExceptionYamlCpp::ErrorCodeEnum& errcode, const std::string& msg)
    : ExceptionAbstract(errcode, msg)
{
}

ExceptionYamlCpp::ExceptionYamlCpp(
    const ExceptionYamlCpp::ErrorCodeEnum& errcode,
    const std::string& msg,
    const char* format,
    ...)
    : ExceptionAbstract(errcode, msg)
{
    va_list args;
    va_start(args, format);

    const unsigned int SIZE = 256;
    char buffer[SIZE];
    vsnprintf(buffer, SIZE, format, args);

    message += buffer;

    va_end(args);
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
