# dynamic_graph_manager
# =====================

Dynamic graph "glue code" responsible for the instanciation of the graph and the
python interpreter. Provide a ROS server in order to create distant ROS clients.

## Intallation:
## ------------
This is a ros package so it should be in a ROS environment.
One can still test its compilation by using the following instruction:

`cd dynamic_graph_manager
mkdir _build
cd _build
cmake ..
make -j8`

## Usage:
## ------

Inherite from the clas DynamicGraphManager and overload the three functions
responsible for the hardware:

`virtual void initialize_hardware_communication_process()`

`virtual void get_sensors_to_map(VectorDGMap&)`

and

`virtual void set_motor_controls_from_map(const VectorDGMap&)`

## Documentation:
## --------------

See demo and unnit tests for more informatino on the API.
Doxygen informations are available by calling:
`make doc`
For more informations check this wiki page:
https://atlas.is.localnet/confluence/display/AMDW/Dynamic-Graph+Framework
