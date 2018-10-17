#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>


#include "single_motor/dgm_single_motor.hh"

namespace dynamic_graph_demo {
    /**
       \brief Feedback controller for an inverted pendulum
       This class implements a feedback control for the inverted pendulum
       represented by class InvertedPendulum
    */
    class MotorController : public dynamicgraph::Entity
    {
    public:
      /**
	 \brief Constructor by name
      */
      MotorController(const std::string& inName);

      ~MotorController();

      /// Each entity should provide the name of the class it belongs to
      virtual const std::string& getClassName (void) const {
        return CLASS_NAME;
      }

      /// Header documentation of the python class
      virtual std::string getDocString () const {
	       return "Basic motor controller\n";
      }
      /**
	  \name Parameters
	  @{
      */
      /**
	 \brief Get feedback gain
      */
      void setGain (const dynamicgraph::Matrix& inGain) {
         gain_ = inGain;
      }

      /**
	 \brief Get feedback gain
      */
      dynamicgraph::Matrix getGain () const {
         return gain_;
      }

      /**
       * \brief Set desired position.
       */
      void setDesired(const dynamicgraph::Matrix& inDesired) {
         desired_ = inDesired;
      }

      /**
	 @}
      */

    protected:
      /*
	\brief Class name
      */
      static const std::string CLASS_NAME;

    private:
      /**
	 Compute the control law
      */
      dynamicgraph::Vector& computeMotorControl(dynamicgraph::Vector& force, const int& inTime);

      /**
	 \brief State of the inverted pendulum
      */
      dynamicgraph::SignalPtr < dynamicgraph::Vector, int> stateSIN;
      /**
	 \brief Force computed by the control law
      */
      dynamicgraph::SignalTimeDependent < dynamicgraph::Vector, int > torqueSOUT;

      /// \brief Gain of the controller
      dynamicgraph::Matrix gain_;

      /// \brief Gain of the controller
      dynamicgraph::Matrix desired_;
    };
} // namespace dynamic_graph_demo
