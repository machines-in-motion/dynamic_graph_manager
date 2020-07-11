/**
 * @file exception-dynamic.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef DYNAMIC_EXCEPTION_HH
#define DYNAMIC_EXCEPTION_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic_graph_manager/exception/exception-abstract.hpp>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph
{
/** \brief ExceptionDynamic */
class ExceptionDynamic : public ExceptionAbstract
{
public:
    enum ErrorCodeEnum
    {
        GENERIC = ExceptionAbstract::DYNAMIC

        ,
        CANT_DESTROY_SIGNAL,
        JOINT_RANK,
        DYNAMIC_JRL,
        JOINT_SIZE,
        INTEGRATION
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName(void) const
    {
        return EXCEPTION_NAME;
    }

public:
    ExceptionDynamic(const ExceptionDynamic::ErrorCodeEnum& errcode,
                     const std::string& msg = "");
    ExceptionDynamic(const ExceptionDynamic::ErrorCodeEnum& errcode,
                     const std::string& msg,
                     const char* format,
                     ...);
    virtual ~ExceptionDynamic(void) throw()
    {
    }
};
} /* namespace dynamic_graph */

#endif /* #ifndef DYNAMIC_EXCEPTION_HH */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
