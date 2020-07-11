/**
 * @file exception-task.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef EXCEPTION_TASK_HH
#define EXCEPTION_TASK_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic_graph_manager/exception/exception-abstract.hpp>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph
{
/** \brief ExceptionTask */
class ExceptionTask : public ExceptionAbstract
{
public:
    enum ErrorCodeEnum
    {
        GENERIC = ExceptionAbstract::TASK,
        EMPTY_LIST,
        NON_ADEQUATE_FEATURES,
        MATRIX_SIZE,
        BOUND_TYPE,
        PARSER_MULTI_BOUND
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName(void) const
    {
        return EXCEPTION_NAME;
    }

    ExceptionTask(const ExceptionTask::ErrorCodeEnum& errcode,
                  const std::string& msg = "");
    ExceptionTask(const ExceptionTask::ErrorCodeEnum& errcode,
                  const std::string& msg,
                  const char* format,
                  ...);
    virtual ~ExceptionTask(void) throw()
    {
    }
};
}  // namespace dynamic_graph

#endif /* #ifndef EXCEPTION_TASK_HH */
