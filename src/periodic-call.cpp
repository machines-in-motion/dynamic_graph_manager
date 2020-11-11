/**
 * @file periodic-call.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/debug.h>
#include <dynamic-graph/exception-factory.h>
#include <dynamic-graph/pool.h>
#include <algorithm>
#include <dynamic-graph/python/interpreter.hh>

#include <dynamic_graph_manager/exception/exception-tools.hpp>
#include <dynamic_graph_manager/periodic-call.hpp>

using namespace std;
using namespace dynamic_graph_manager;
namespace dg = dynamicgraph;

PeriodicCall::PeriodicCall(void)
    : signal_map_(), cmd_list_(), inner_time_(0), py_interpreter_(nullptr)
{
}

void PeriodicCall::addSignal(const std::string& name, dg::SignalBase<int>& sig)
{
    signal_map_[name] = SignalToCall(&sig);
    return;
}

void PeriodicCall::addSignal(const std::string& sigpath)
{
    std::istringstream sig_iss(sigpath);
    dg::SignalBase<int>& signal =
        ::dg::PoolStorage::getInstance()->getSignal(sig_iss);
    addSignal(sigpath, signal);
    return;
}

void PeriodicCall::addDownsampledSignal(const std::string& name,
                                        dg::SignalBase<int>& sig,
                                        const unsigned int& downsamplingFactor)
{
    signal_map_[name] = SignalToCall(&sig, downsamplingFactor);
    return;
}

void PeriodicCall::addDownsampledSignal(const std::string& sigpath,
                                        const unsigned int& downsamplingFactor)
{
    std::istringstream sig_iss(sigpath);
    dg::SignalBase<int>& signal =
        ::dg::PoolStorage::getInstance()->getSignal(sig_iss);
    addDownsampledSignal(sigpath, signal, downsamplingFactor);
    return;
}

void PeriodicCall::rmSignal(const std::string& name)
{
    signal_map_.erase(name);
    return;
}

void PeriodicCall::addCmd(const std::string& cmdLine)
{
    cmd_list_.push_back(cmdLine);
    return;
}

void PeriodicCall::rmCmd(const std::string& args)
{
    CmdListType::iterator iter =
        std::find(cmd_list_.begin(), cmd_list_.end(), args);
    if (cmd_list_.end() != iter)
    {
        cmd_list_.erase(iter);
    }
}

void PeriodicCall::runSignals(const int& t)
{
    for (SignalMapType::iterator iter = signal_map_.begin();
         signal_map_.end() != iter;
         ++iter)
    {
        if (static_cast<unsigned int>(t) % iter->second.down_sampling_factor_ ==
            0)
        {
            (*iter).second.signal_->recompute(t);
        }
    }
    return;
}

void PeriodicCall::runCmds(void)
{
    if (nullptr == py_interpreter_)
    {
        ExceptionTools(ExceptionTools::PY_SHELL_PTR,
                       "Python interpreter not set.");
    }

    std::string result, out, err;

    for (CmdListType::const_iterator iter = cmd_list_.begin();
         cmd_list_.end() != iter;
         ++iter)
    {
        py_interpreter_->python(*iter, result, out, err);
    }
    return;
}

void PeriodicCall::run(const int& t)
{
    runSignals(t);
    runCmds();
    return;
}

void PeriodicCall::display(std::ostream& os) const
{
    os << "  (t=" << inner_time_ << ")" << endl;

    os << " -> SIGNALS:" << endl;
    for (SignalMapType::const_iterator iter = signal_map_.begin();
         signal_map_.end() != iter;
         ++iter)
    {
        os << " - " << (*iter).first << endl;
    }

    os << " -> CMDS:" << endl;
    for (CmdListType::const_iterator iter = cmd_list_.begin();
         cmd_list_.end() != iter;
         ++iter)
    {
        os << " - " << (*iter) << endl;
    }
}
