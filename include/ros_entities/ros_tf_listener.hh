#ifndef DYNAMIC_GRAPH_ROS_TF_LISTENER_HH
# define DYNAMIC_GRAPH_ROS_TF_LISTENER_HH

# include <boost/bind.hpp>

# include <tf/transform_listener.h>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal.h>
# include <dynamic-graph/command-bind.h>

# include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
  class RosTfListener;

  namespace internal
  {
    struct TransformListenerData {
      typedef Signal<sot::MatrixHomogeneous, int> signal_t;

      tf::TransformListener& listener;
      const std::string toFrame, fromFrame;
      tf::StampedTransform transform;
      signal_t signal;

      TransformListenerData (tf::TransformListener& l,
          const std::string& to, const std::string& from,
          const std::string& signame)
        : listener (l)
        , toFrame (to)
        , fromFrame (from)
        , signal (signame)
      {
        signal.setFunction (boost::bind(&TransformListenerData::getTransform, this, _1, _2));
      }

      sot::MatrixHomogeneous& getTransform (sot::MatrixHomogeneous& res, int time)
      {
        static const ros::Time rosTime(0);
        try {
          listener.lookupTransform (toFrame, fromFrame, rosTime, transform);
        } catch (const tf::TransformException& ex) {
          res.setIdentity();
          ROS_ERROR("Enable to get transform at time %i: %s",time,ex.what());
          return res;
        }
        for (sot::MatrixHomogeneous::Index r = 0; r < 3; ++r) {
          for (sot::MatrixHomogeneous::Index c = 0; c < 3; ++c)
            res.linear ()(r,c) = transform.getBasis().getRow(r)[c];
          res.translation()[r] = transform.getOrigin()[r];
        }
        return res;
      }
    };
  } // end of internal namespace.

  class RosTfListener : public Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();

    public:
      typedef internal::TransformListenerData TransformListenerData; 

      RosTfListener (const std::string& name) : Entity (name)
      {
        std::string docstring =
          "\n"
          "  Add a signal containing the transform between two frames.\n"
          "\n"
          "  Input:\n"
          "    - to  : frame name\n"
          "    - from: frame name,\n"
          "    - signalName: the signal name in dynamic-graph"
          "\n";
        addCommand ("add",
            command::makeCommandVoid3(*this, &RosTfListener::add, docstring));
      }

      ~RosTfListener ()
      {
        for (Map_t::const_iterator _it = listenerDatas.begin(); _it != listenerDatas.end(); ++_it)
          delete _it->second;
      }

      void add (const std::string& to, const std::string& from, const std::string& signame)
      {
        if (listenerDatas.find(signame) != listenerDatas.end())
          throw std::invalid_argument ("A signal " + signame
              + " already exists in RosTfListener " + getName());

        boost::format signalName ("RosTfListener(%1%)::output(MatrixHomo)::%2%");
        signalName % getName () % signame;

        TransformListenerData* tld = new TransformListenerData (
            listener, to, from, signalName.str());
        signalRegistration (tld->signal);
        listenerDatas[signame] = tld;
      }

    private:
      typedef std::map<std::string, TransformListenerData*> Map_t;
      Map_t listenerDatas;
      tf::TransformListener listener;
  };
} // end of namespace dynamicgraph.

#endif // DYNAMIC_GRAPH_ROS_TF_LISTENER_HH
