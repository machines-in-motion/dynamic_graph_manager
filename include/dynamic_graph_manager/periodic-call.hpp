/**
 * @file periodic-call.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef PERIODICCALL_HH
#define PERIODICCALL_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-base.h>
#include <dynamic-graph/python/interpreter.hh>

#include <list>
#include <map>
#include <string>

namespace dynamic_graph_manager
{
/*!
  \class PeriodicCall
*/
class PeriodicCall
{
protected:
    struct SignalToCall
    {
        dynamicgraph::SignalBase<int>* signal_;
        unsigned int down_sampling_factor_;

        SignalToCall()
        {
            signal_ = NULL;
            down_sampling_factor_ = 1;
        }

        SignalToCall(dynamicgraph::SignalBase<int>* s, unsigned int df = 1)
        {
            signal_ = s;
            down_sampling_factor_ = df;
        }
    };

    typedef std::map<std::string, SignalToCall> SignalMapType;
    SignalMapType signal_map_;

    typedef std::list<std::string> CmdListType;
    CmdListType cmd_list_;

    int inner_time_;
    dynamicgraph::python::Interpreter* py_interpreter_;

    /* --- FUNCTIONS
     * ------------------------------------------------------------ */
public:
    PeriodicCall(void);
    virtual ~PeriodicCall(void)
    {
    }

    void addDownsampledSignal(const std::string& name,
                              dynamicgraph::SignalBase<int>& sig,
                              const unsigned int& downsamplingFactor);
    void addDownsampledSignal(const std::string& sigpath,
                              const unsigned int& downsamplingFactor);

    void addSignal(const std::string& name, dynamicgraph::SignalBase<int>& sig);
    void addSignal(const std::string& args);
    void rmSignal(const std::string& name);

    void addCmd(const std::string& args);
    void rmCmd(const std::string& args);

    void runSignals(const int& t);
    void runCmds(void);
    void run(const int& t);

    void clear(void)
    {
        signal_map_.clear();
        cmd_list_.clear();
    }

    void display(std::ostream& os) const;
    bool commandLine(const std::string& cmdLine,
                     std::istringstream& cmdArgs,
                     std::ostream& os);
    void addSpecificCommands(dynamicgraph::Entity& ent,
                             dynamicgraph::Entity::CommandMap_t& commap,
                             const std::string& prefix = "");

    void setPyInterpreter(dynamicgraph::python::Interpreter* py_interpreter)
    {
        py_interpreter_ = py_interpreter;
    }
};

}  // namespace dynamic_graph_manager

#endif  // #ifndef PERIODICCALL_HH
