/**
 * \file exception-factory.hh
 * \brief An exception class that provides usefull information in case of bug
 * catch.
 * \author Maximilien Naveau
 * \date 2018
 */

#ifndef EXCEPTION_FACTORY_HH
#define EXCEPTION_FACTORY_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <dynamic_graph_manager/exception/exception-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph {

  /**
   * @brief The ExceptionFactory class
   */
  class ExceptionFactory: public ExceptionAbstract
  {
  public:

    enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::FACTORY
      ,UNREFERED_OBJECT
      ,UNREFERED_SIGNAL
      ,UNREFERED_FUNCTION
      ,DYNAMIC_LOADING
      ,SIGNAL_CONFLICT
      ,FUNCTION_CONFLICT
      ,OBJECT_CONFLICT
      ,SYNTAX_ERROR    // j' aime bien FATAL_ERROR aussi faut que je la case qq part...
      ,READ_FILE
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName( void )const{ return ExceptionFactory::EXCEPTION_NAME; }

    ExceptionFactory ( const ExceptionFactory::ErrorCodeEnum& errcode,
                       const std::string & msg = "" );
    ExceptionFactory ( const ExceptionFactory::ErrorCodeEnum& errcode,
                       const std::string & msg,const char* format, ... );
    virtual ~ExceptionFactory( void ) throw() {}

  };
} /* namespace dynamic_graph */


#endif /* #ifndef EXCEPTION_FACTORY_HH */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
