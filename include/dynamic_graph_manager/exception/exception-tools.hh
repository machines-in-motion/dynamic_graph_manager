/**
 * @file exception-tools.hh
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-05-22
 */

#ifndef TOOLS_EXCEPTION_HH
#define TOOLS_EXCEPTION_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <dynamic_graph_manager/exception/exception-abstract.hh>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph {

  /* \class ExceptionTools
     */
  class ExceptionTools :public ExceptionAbstract
  {
  public:
    enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::TOOLS

      ,CORBA
      ,KALMAN_SIZE
      ,PY_SHELL_PTR
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName() const {
      return EXCEPTION_NAME;
    }

  public:

    ExceptionTools ( const ExceptionTools::ErrorCodeEnum& errcode,
                     const std::string & msg = "" );
    ExceptionTools( const ExceptionTools::ErrorCodeEnum& errcode,
                    const std::string & msg,const char* format, ... );
    virtual ~ExceptionTools( void ) throw() {}
  };

} // namespace dynamicgraph



#endif /* #ifndef TOOLS_EXCEPTION_HH */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
