/**
 * \file exception-factory.cpp
 * \brief Implement exception for debugging use.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file implements the input and output of the DynamicGraph
 */

#include <stdarg.h>
#include <cstdio>

#include <dynamic-graph/debug.h>

#include <dynamic_graph_manager/exception/exception-factory.hh>


using namespace dynamic_graph;
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionFactory::EXCEPTION_NAME = "Factory";

ExceptionFactory::ExceptionFactory (
    const ExceptionFactory::ErrorCodeEnum& errcode,
    const std::string & msg ):
  ExceptionAbstract(errcode,msg)
{
  dgDEBUGF( 15,"Created with message <%s>.",msg.c_str());
  dgDEBUG( 1) <<"Created with message <%s>."<<msg<<std::endl;
}

ExceptionFactory::
ExceptionFactory ( const ExceptionFactory::ErrorCodeEnum& errcode,
                   const std::string & msg, const char* format, ... ):
  ExceptionAbstract(errcode,msg)
{
  va_list args;
  va_start(args,format);

  const unsigned int SIZE = 256;
  char  buffer[SIZE];
  vsnprintf(buffer, SIZE, format, args);

  dgDEBUG(15) <<"Created "<<" with message <"
	       <<msg<<"> and buffer <"<<buffer<<">. "<<std::endl;

  message += buffer;

  va_end(args);

  dgDEBUG(1) << "Throw exception " << EXCEPTION_NAME << "[#" << errcode<<"]: "
	      <<"<"<< message << ">."<<std::endl;

}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
