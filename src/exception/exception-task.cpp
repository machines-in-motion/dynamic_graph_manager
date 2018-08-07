/**
 * \file exception-task.cpp
 * \brief An exception class that provides usefull information in case of bug
 * catch
 * \author Maximilien Naveau
 * \date 2018
 */

#include <dynamic_graph_manager/exception/exception-task.hh>
#include <stdarg.h>
#include <cstdio>


using namespace dynamic_graph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionTask::EXCEPTION_NAME = "Task";

ExceptionTask::
ExceptionTask ( const ExceptionTask::ErrorCodeEnum& errcode,
		   const std::string & msg )
  :ExceptionAbstract(errcode,msg)
{
}

ExceptionTask::
ExceptionTask ( const ExceptionTask::ErrorCodeEnum& errcode,
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
