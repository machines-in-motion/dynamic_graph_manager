/**
 * @file exception-feature.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-05-22
 */

#include <dynamic_graph_manager/exception/exception-feature.hh>
#include <stdarg.h>
#include <cstdio>

using namespace dynamic_graph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionFeature::EXCEPTION_NAME = "Feature";

ExceptionFeature::
ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
		      const std::string & msg )
  :ExceptionAbstract(errcode,msg)
{
}

ExceptionFeature::
ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... )
  :ExceptionAbstract(errcode,msg)
{
  va_list args;
  va_start(args,format);

  const unsigned int SIZE = 256;
  char  buffer[SIZE];
  vsnprintf(buffer,SIZE,format,args);

  message = buffer;

  va_end(args);
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
