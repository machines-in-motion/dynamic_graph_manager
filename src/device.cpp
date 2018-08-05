/**
 * \file device.cpp
 * \brief The robot entity
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file implements the input and output of the DynamicGraph
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <Eigen/Geometry>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/linear-algebra.h>

#include <dynamic_graph_manager/device.hh>
#include <dynamic_graph_manager/debug/debug.hh>

using namespace std;
using namespace dynamic_graph;

const std::string dynamic_graph::Device::CLASS_NAME = "Device";

Device::
~Device( )
{
  for( unsigned int i=0; i<4; ++i ) {
    delete forcesSOUT[i];
  }
}

Device::
Device( const std::string& n )
  :Entity(n)
  ,state_(6)
  ,robotState_("Device(" + n + ")::output(vector)::robotState")
  ,robotVelocity_("Device(" + n + ")::output(vector)::robotVelocity")
  ,vel_controlInit_(false)
  ,controlInputType_(CONTROL_INPUT_ONE_INTEGRATION)
  ,controlSIN( NULL,"Device("+n+")::input(double)::control" )
  //,attitudeSIN(NULL,"Device::input(matrixRot)::attitudeIN")
  ,attitudeSIN(NULL,"Device::input(vector3)::attitudeIN")
  ,zmpSIN(NULL,"Device::input(vector3)::zmp")
  ,stateSOUT( "Device("+n+")::output(vector)::state" )
  ,velocitySOUT( "Device("+n+")::output(vector)::velocity"  )
  ,attitudeSOUT( "Device("+n+")::output(matrixRot)::attitude" )
  ,pseudoTorqueSOUT( "Device::output(vector)::ptorque" )
  ,previousControlSOUT( "Device("+n+")::output(vector)::previousControl" )
  ,motorcontrolSOUT( "Device("+n+")::output(vector)::motorcontrol" )
  ,ZMPPreviousControllerSOUT( "Device("+n+")::output(vector)::zmppreviouscontroller" ), ffPose_(),
    forceZero6 (6)
{
  forceZero6.fill (0);
  /* --- SIGNALS --- */
  for( int i=0;i<4;++i ){ withForceSignals[i] = false; }
  forcesSOUT[0] =
      new Signal<Vector, int>("OpenHRP::output(vector6)::forceRLEG");
  forcesSOUT[1] =
      new Signal<Vector, int>("OpenHRP::output(vector6)::forceLLEG");
  forcesSOUT[2] =
      new Signal<Vector, int>("OpenHRP::output(vector6)::forceRARM");
  forcesSOUT[3] =
      new Signal<Vector, int>("OpenHRP::output(vector6)::forceLARM");

  signalRegistration( controlSIN<<stateSOUT<<robotState_<<robotVelocity_
                      <<velocitySOUT<<attitudeSOUT
                      <<attitudeSIN<<zmpSIN <<*forcesSOUT[0]<<*forcesSOUT[1]
                      <<*forcesSOUT[2]<<*forcesSOUT[3] <<previousControlSOUT
                      <<pseudoTorqueSOUT << motorcontrolSOUT << ZMPPreviousControllerSOUT );
  state_.fill(.0); stateSOUT.setConstant( state_ );

  velocity_.resize(state_.size()); velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );

  /* --- Commands --- */
  {
    std::string docstring;
    /* Command setStateSize. */
    docstring =
        "\n"
        "    Set size of state vector\n"
        "\n";
    addCommand("resize",
               new command::Setter<Device, unsigned int>
               (*this, &Device::setStateSize, docstring));
    docstring =
        "\n"
        "    Set state vector value\n"
        "\n";
    addCommand("set",
               new command::Setter<Device, Vector>
               (*this, &Device::setState, docstring));

    docstring =
        "\n"
        "    Set velocity vector value\n"
        "\n";
    addCommand("setVelocity",
               new command::Setter<Device, Vector>
               (*this, &Device::setVelocity, docstring));

    void(Device::*setRootPtr)(const Matrix&) = &Device::setRoot;
    docstring
        = command::docCommandVoid1("Set the root position.",
                                   "matrix homogeneous");
    addCommand("setRoot",
               command::makeCommandVoid1(*this,setRootPtr,
                                         docstring));

    /* Second Order Integration set. */
    docstring =
        "\n"
        "    Set the position calculous starting from  \n"
        "    acceleration measure instead of velocity \n"
        "\n";

    addCommand("setSecondOrderIntegration",
               command::makeCommandVoid0(*this,&Device::setSecondOrderIntegration,
                                         docstring));

    /* SET of control input type. */
    docstring =
        "\n"
        "    Set the type of control input which can be  \n"
        "    acceleration, velocity, or position\n"
        "\n";

    addCommand("setControlInputType",
               new command::Setter<Device,string>
               (*this, &Device::setControlInputType, docstring));

    // Handle commands and signals called in a synchronous way.
    periodic_call_before_.addSpecificCommands(*this, commandMap, "before.");
    periodic_call_after_.addSpecificCommands(*this, commandMap, "after.");

  }
}

void Device::
setStateSize( const unsigned int& size )
{
  state_.resize(size); state_.fill( .0 );
  stateSOUT .setConstant( state_ );
  previousControlSOUT.setConstant( state_ );
  pseudoTorqueSOUT.setConstant( state_ );
  motorcontrolSOUT .setConstant( state_ );

  Device::setVelocitySize(size);

  Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );
}

void Device::
setVelocitySize( const unsigned int& size )
{
  velocity_.resize(size);
  velocity_.fill(.0);
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setState( const Vector& st )
{
  state_ = st;
  stateSOUT .setConstant( state_ );
  motorcontrolSOUT .setConstant( state_ );
}

void Device::
setVelocity( const Vector& vel )
{
  velocity_ = vel;
  velocitySOUT .setConstant( velocity_ );
}

void Device::
setRoot( const Matrix & root )
{
  Eigen::Matrix4d _matrix4d(root);
  MatrixHomogeneous _root(_matrix4d);
  setRoot( _root );
}

void Device::
setRoot( const MatrixHomogeneous & worldMwaist )
{
  VectorRollPitchYaw r = (worldMwaist.linear().eulerAngles(2,1,0)).reverse();
  Vector q = state_;
  q = worldMwaist.translation(); // abusive ... but working.
  for( unsigned int i=0;i<3;++i ) q(i+3) = r(i);
}

void Device::
setSecondOrderIntegration()
{
  controlInputType_ = CONTROL_INPUT_TWO_INTEGRATION;
  velocity_.resize(state_.size());
  velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setNoIntegration()
{
  controlInputType_ = CONTROL_INPUT_NO_INTEGRATION;
  velocity_.resize(state_.size());
  velocity_.setZero();
  velocitySOUT.setConstant( velocity_ );
}

void Device::
setControlInputType(const std::string& cit)
{
  for(int i=0; i<CONTROL_INPUT_SIZE; i++)
    if(cit==ControlInput_s[i])
    {
      controlInputType_ = (ControlInput)i;
      sotDEBUG(25)<<"Control input type: "<<ControlInput_s[i]<<endl;
      return;
    }
  sotDEBUG(25)<<"Unrecognized control input type: "<<cit<<endl;
}

void Device::
increment( const double & dt )
{
  int time = stateSOUT.getTime();
  sotDEBUG(25) << "Time : " << time << std::endl;

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodic_call_before_.run(time+1);
  }
  catch (std::exception& e)
  {
    std::cerr
        << "exception caught while running periodical commands (before): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    std::cerr
        << "exception caught while running periodical commands (before): "
        << str << std::endl;
  }
  catch (...)
  {
    std::cerr
        << "unknown exception caught while"
        << " running periodical commands (before)" << std::endl;
  }


  /* Force the recomputation of the control. */
  controlSIN( time );
  sotDEBUG(25) << "u" <<time<<" = " << controlSIN.accessCopy() << endl;

  /* Integration of numerical values. This function is virtual. */
  integrate( dt );
  sotDEBUG(25) << "q" << time << " = " << state_ << endl;

  /* Position the signals corresponding to sensors. */
  stateSOUT .setConstant( state_ ); stateSOUT.setTime( time+1 );
  //computation of the velocity signal
  if( controlInputType_==CONTROL_INPUT_TWO_INTEGRATION )
  {
    velocitySOUT.setConstant( velocity_ );
    velocitySOUT.setTime( time+1 );
  }
  else if (controlInputType_==CONTROL_INPUT_ONE_INTEGRATION)
  {
    velocitySOUT.setConstant( controlSIN.accessCopy() );
    velocitySOUT.setTime( time+1 );
  }
  for( int i=0;i<4;++i ){
    if(  !withForceSignals[i] ) forcesSOUT[i]->setConstant(forceZero6);
  }
  Vector zmp(3); zmp.fill( .0 );
  ZMPPreviousControllerSOUT .setConstant( zmp );

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodic_call_after_.run(time+1);
  }
  catch (std::exception& e)
  {
    std::cerr
        << "exception caught while running periodical commands (after): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    std::cerr
        << "exception caught while running periodical commands (after): "
        << str << std::endl;
  }
  catch (...)
  {
    std::cerr
        << "unknown exception caught while"
        << " running periodical commands (after)" << std::endl;
  }


  // Others signals.
  motorcontrolSOUT .setConstant( state_ );
}

void Device::display ( std::ostream& os ) const
{
  os << name_ << ": " << state_ << std::endl;
}
