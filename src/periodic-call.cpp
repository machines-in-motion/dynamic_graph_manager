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

#include <dynamic_graph_manager/exception/exception-tools.hh>
#include <dynamic_graph_manager/periodic-call.hh>

using namespace std;
using namespace dynamic_graph;
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

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

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

static std::string readLineStr(istringstream& args)
{
    stringbuf* pbuf = args.rdbuf();
    const std::streamsize size = pbuf->in_avail();
    char* buffer = new char[size + 1];
    pbuf->sgetn(buffer, size);

    buffer[size] = '\0';
    std::string res(buffer);
    delete[] buffer;
    return res;
}

bool PeriodicCall::commandLine(const std::string& cmdLine,
                               std::istringstream& cmdArgs,
                               std::ostream& os)
{
    if (cmdLine == "help")
    {
        os << "PeriodicCall:" << endl
           << "  - addSignal/rmSignal  <int> " << endl
           << "  - addCmd/rmCmd  " << endl
           << "  - runSignal/runCmd " << endl
           << "  - run" << endl;
    }
    else if (cmdLine == "addSignal")
    {
        std::string sigpath;
        cmdArgs >> std::skipws >> sigpath;
        addSignal(sigpath);
    }
    else if (cmdLine == "rmSignal")
    {
        std::string sigpath;
        cmdArgs >> std::skipws >> sigpath;
        rmSignal(sigpath);
    }
    else if (cmdLine == "runSignals")
    {
        runSignals(inner_time_++);
    }

    else if (cmdLine == "addCmd")
    {
        addCmd(readLineStr(cmdArgs));
    }
    else if (cmdLine == "rmCmd")
    {
        rmCmd(readLineStr(cmdArgs));
    }
    else if (cmdLine == "runCmds")
    {
        runCmds();
    }

    else if (cmdLine == "run")
    {
        run(inner_time_++);
    }
    else if (cmdLine == "clear")
    {
        clear();
    }
    else if (cmdLine == "print")
    {
        display(os);
    }
    else
    {
        return false;
    }
    return true;
}

#define ADD_COMMAND(name, def)                                              \
    if (commandMap.count(prefix + name) != 0)                               \
    {                                                                       \
        DG_THROW dg::ExceptionFactory(                                      \
            dg::ExceptionFactory::OBJECT_CONFLICT,                          \
            "Command " + prefix + name + " already registered in Entity."); \
    }                                                                       \
    commandMap.insert(std::make_pair(prefix + name, def))

void PeriodicCall::addSpecificCommands(dg::Entity& ent,
                                       dg::Entity::CommandMap_t& commandMap,
                                       const std::string& prefix)
{
    using namespace dg::command;

    /* Explicit typage to help the compiler. */
    boost::function<void(const std::string&)>
        addSignal = boost::bind(&PeriodicCall::addSignal, this, _1),
        rmSignal = boost::bind(&PeriodicCall::rmSignal, this, _1);
    boost::function<void(const std::string&, const unsigned int&)>
        addDownsampledSignal =
            boost::bind(&PeriodicCall::addDownsampledSignal, this, _1, _2);
    boost::function<void(void)> clear = boost::bind(&PeriodicCall::clear, this);
    boost::function<void(std::ostream&)> disp =
        boost::bind(&PeriodicCall::display, this, _1);

    ADD_COMMAND(
        "addSignal",
        makeCommandVoid1(ent,
                         addSignal,
                         docCommandVoid1("Add the signal to the refresh list",
                                         "string (sig name)")));
    ADD_COMMAND(
        "addDownsampledSignal",
        makeCommandVoid2(
            ent,
            addDownsampledSignal,
            docCommandVoid2("Add the signal to the refresh list",
                            "string (sig name)",
                            "unsigned int (downsampling factor, 1 means every "
                            "time, 2 means every other time, etc...")));
    ADD_COMMAND("rmSignal",
                makeCommandVoid1(
                    ent,
                    rmSignal,
                    docCommandVoid1("Remove the signal to the refresh list",
                                    "string (sig name)")));
    ADD_COMMAND(
        "clear",
        makeCommandVoid0(
            ent,
            clear,
            docCommandVoid0(
                "Clear all signals and commands from the refresh list.")));

    ADD_COMMAND("disp",
                makeCommandVerbose(
                    ent,
                    disp,
                    docCommandVerbose(
                        "Print the list of to-refresh signals and commands.")));
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
