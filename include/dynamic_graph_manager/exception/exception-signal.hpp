/**
 * @file exception-signal.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef SIGNAL_EXCEPTION_HH
#define SIGNAL_EXCEPTION_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic_graph_manager/exception/exception-abstract.hpp>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph
{
/* \class ExceptionSignal */
class ExceptionSignal : public ExceptionAbstract
{
public:
    enum ErrorCodeEnum
    {
        GENERIC = ExceptionAbstract::SIGNAL

        ,
        READWRITE_LOCK,
        COPY_NOT_INITIALIZED,
        NOT_INITIALIZED,
        PLUG_IMPOSSIBLE,
        SET_IMPOSSIBLE,
        BAD_CAST
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName(void) const
    {
        return EXCEPTION_NAME;
    }

public:
    ExceptionSignal(const ExceptionSignal::ErrorCodeEnum& errcode,
                    const std::string& msg = "");
    ExceptionSignal(const ExceptionSignal::ErrorCodeEnum& errcode,
                    const std::string& msg,
                    const char* format,
                    ...);
    virtual ~ExceptionSignal(void) throw()
    {
    }
};
} /* namespace dynamic_graph */

#endif /* #ifndef SIGNAL_EXCEPTION_HH */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
