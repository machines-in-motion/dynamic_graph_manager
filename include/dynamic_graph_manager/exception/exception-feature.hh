/**
 * \file exception-feature.hh
 * \brief An exception class that provides usefull information in case of bug
 * catch
 * \author Maximilien Naveau
 * \date 2018
 */

#ifndef EXCEPTION_FEATURE_HH
#define EXCEPTION_FEATURE_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <dynamic_graph_manager/exception/exception-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph {

  /* \class ExceptionFeature
 */
  class ExceptionFeature
      :public ExceptionAbstract

  {
  public:

    enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::FEATURE
      ,BAD_INIT
      ,UNCOMPATIBLE_SIZE
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName( void ) const { return ExceptionFeature::EXCEPTION_NAME; }

    ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
                       const std::string & msg = "" );

    ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
                       const std::string & msg,const char* format, ... );

    virtual ~ExceptionFeature( void ) throw() {}
  };
} /* namespace dynamicgraph */

#endif /* #ifndef EXCEPTION_FEATURE_HH */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
