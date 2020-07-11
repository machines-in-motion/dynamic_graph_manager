/**
 * @file exception-yaml-cpp.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef YAML_CPP_EXCEPTION_HPP
#define YAML_CPP_EXCEPTION_HPP

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic_graph_manager/exception/exception-abstract.hpp>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph
{
/* \class ExceptionYamlCpp
 */
class ExceptionYamlCpp : public ExceptionAbstract
{
public:
    enum ErrorCodeEnum
    {
        GENERIC = ExceptionAbstract::YAML_CPP_PARSING,
        PARSING_BOOL,
        PARSING_FLOAT,
        PARSING_DOUBLE,
        PARSING_EIGEN,
        PARSING_STRING,
        PARSING_UNSIGNED
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName() const
    {
        return EXCEPTION_NAME;
    }

public:
    ExceptionYamlCpp(const ExceptionYamlCpp::ErrorCodeEnum& errcode,
                     const std::string& msg = "");
    ExceptionYamlCpp(const ExceptionYamlCpp::ErrorCodeEnum& errcode,
                     const std::string& msg,
                     const char* format,
                     ...);
    virtual ~ExceptionYamlCpp(void) throw()
    {
    }
};

}  // namespace dynamic_graph

#endif /* #ifndef YAML_CPP_EXCEPTION_HPP */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
