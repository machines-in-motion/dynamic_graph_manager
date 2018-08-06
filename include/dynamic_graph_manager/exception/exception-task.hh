/**
 * \file exception-task.cpp
 * \brief An exception class that provides usefull information in case of bug
 * catch
 * \author Maximilien Naveau
 * \date 2018
 */

#ifndef EXCEPTION_TASK_HH
#define EXCEPTION_TASK_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <dynamic_graph_manager/exception/exception-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph {
    /** \brief ExceptionTask */
    class ExceptionTask: public ExceptionAbstract
    {
    public:

      enum ErrorCodeEnum
      {
        GENERIC = ExceptionAbstract::TASK
        ,EMPTY_LIST
        ,NON_ADEQUATE_FEATURES
        ,MATRIX_SIZE
        ,BOUND_TYPE
        ,PARSER_MULTI_BOUND
      };

      static const std::string EXCEPTION_NAME;
      virtual const std::string& getExceptionName( void ) const { return EXCEPTION_NAME; }

      ExceptionTask ( const ExceptionTask::ErrorCodeEnum& errcode,
                      const std::string & msg = "" );
      ExceptionTask( const ExceptionTask::ErrorCodeEnum& errcode,
                     const std::string & msg,const char* format, ... );
      virtual ~ExceptionTask( void ) throw() {}

    };
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_EXCEPTION_TASK_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
