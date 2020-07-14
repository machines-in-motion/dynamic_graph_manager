/**
 * @file exception-abstract.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */
#ifndef ABSTRACT_EXCEPTION_HH
#define ABSTRACT_EXCEPTION_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Classes standards. */
#include <exception>
#include <iostream> /* Classe ostream.    */
#include <string>   /* Classe string.     */

// Uncomment this macros to have lines parameter on the throw display
// #define EXCEPTION_PASSING_PARAM

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamic_graph_manager
{
/**
 * @brief The ExceptionAbstract class
 */
class ExceptionAbstract : public std::exception
{
public:
    enum ExceptionEnum
    {
        ABSTRACT = 0,
        SIGNAL = 100,
        TASK = 200,
        FEATURE = 300,
        FACTORY = 400,
        DYNAMIC = 500,
        TRACES = 600,
        TOOLS = 700,
        PATTERN_GENERATOR = 800,
        YAML_CPP_PARSING = 900
    };

    static const std::string EXCEPTION_NAME;
    virtual const std::string& getExceptionName(void) const
    {
        return EXCEPTION_NAME;
    }

protected:
    /** Error code.
     * \sa ErrorCodeEnum */
    int code;

    /**  Error message (can be empty). */
    std::string message;

private:
    /**  forbid the empty constructor (private). */
    ExceptionAbstract(void);

public:
    ExceptionAbstract(const int& code, const std::string& msg = "");
    virtual ~ExceptionAbstract(void) throw()
    {
    }

    /**  Access to the error code. */
    int getCode(void);

    /** Reference access to the error message (can be empty). */
    const std::string& getStringMessage(void);

    /** Access to the pointer on the array of  \e char related to the error
     * string. Cannot be  \e NULL.
     */
    const char* getMessage(void);
    const char* what() const throw();

    /** Print the error structure. */
    friend std::ostream& operator<<(std::ostream& os,
                                    const ExceptionAbstract& err);

#ifdef EXCEPTION_PASSING_PARAM
public:
    class Param
    {
    public:
        static const int BUFFER_SIZE = 80;

        const char* functionPTR;
        char function[BUFFER_SIZE];
        int line;
        const char* filePTR;
        char file[BUFFER_SIZE];
        bool pointersSet, set;

    public:
        Param(const int& _line, const char* _function, const char* _file);
        Param(void) : pointersSet(false), set(false)
        {
        }
        Param& initCopy(const Param& p);
    };

protected:
    mutable Param p;

    template <class Exc>
    friend const Exc& operator+(const ExceptionAbstract::Param& p, const Exc& e)
    {
        e.p.initCopy(p);
        return e;
    }
    template <class Exc>
    friend Exc& operator+(const ExceptionAbstract::Param& p, Exc& e)
    {
        e.p.initCopy(p);
        return e;
    }
#endif  //#ifdef EXCEPTION_PASSING_PARAM
};

#define SOT_RETHROW                \
    (const ExceptionAbstract& err) \
    {                              \
        throw err;                 \
    }

#ifdef EXCEPTION_PASSING_PARAM
#define SOT_THROW \
    throw ExceptionAbstract::Param(__LINE__, __FUNCTION__, __FILE__) +
#else  //#ifdef EXCEPTION_PASSING_PARAM
#define DG_THROW throw
#endif  //#ifdef EXCEPTION_PASSING_PARAM

} /* namespace dynamic_graph_manager */

#endif /* #ifndef ABSTRACT_EXCEPTION_HH */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
