/**
 * @file exception-feature.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef EXCEPTION_FEATURE_HH
#define EXCEPTION_FEATURE_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic_graph_manager/exception/exception-abstract.hpp>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph
{
/* \class ExceptionFeature
 */
class ExceptionFeature : public ExceptionAbstract

{
public:
    enum ErrorCodeEnum
    {
        GENERIC = ExceptionAbstract::FEATURE,
        BAD_INIT,
        UNCOMPATIBLE_SIZE
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName(void) const
    {
        return ExceptionFeature::EXCEPTION_NAME;
    }

    ExceptionFeature(const ExceptionFeature::ErrorCodeEnum& errcode,
                     const std::string& msg = "");

    ExceptionFeature(const ExceptionFeature::ErrorCodeEnum& errcode,
                     const std::string& msg,
                     const char* format,
                     ...);

    virtual ~ExceptionFeature(void) throw()
    {
    }
};
}  // namespace dynamic_graph

#endif /* #ifndef EXCEPTION_FEATURE_HH */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
