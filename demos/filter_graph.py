from dynamic_graph import plug
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from dynamic_graph.sot.core.operator import (
    Multiply_double_vector,
    Selec_of_vector,
    Stack_of_vector,
    Substract_of_vector,
)
from dynamic_graph import writeGraph


def create_simple_graph():
    """
    We create a simple graph and create its graphical representation
    """

    # here we create some input to graph. These a constant vector but it could
    # anything coming from the hardware
    centered_slider = VectorConstant("4_robot_sliders")
    centered_slider.sout.value = [0.5, 0.5, 0.5, 0.5]

    # Filter the centered sliders
    # Hence we create a "Finite Impendance Response" filter.
    # the filter is in the following form:
    # out = sum_{i=0}^{N} data_i * alpha_i
    #   - the data_i are the collected elements, their number grows until the
    #     size of the filter is reached.
    #   - the alpha_i are the gains of the filter, they are defined by the
    #     method "setElement(index, value)"
    # in the end here we do an averaging filter on 200 points.
    slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
    filter_size = 200
    slider_filtered.setSize(filter_size)
    for i in range(filter_size):
        slider_filtered.setElement(i, 1.0 / float(filter_size))
    # we plug the centered sliders output to the input of the filter.
    plug(centered_slider.sout, slider_filtered.sin)

    # Now we want the slider to be in [-qref, qref]
    # So we multiply all sliders by a constant which is max_qref.
    scaled_slider = Multiply_double_vector("scaled_slider")
    scaled_slider.sin1.value = 2.0
    plug(slider_filtered.sout, scaled_slider.sin2)

    # Now we need to solve the problem that we have 4 sliders for 8 motors.
    # Hence we will map each slider value to 2 motors.
    state = {}
    for i, leg in enumerate(["fr", "hr", "hl", "fl"]):
        # first of all we define the references for the hip joint:
        state[leg + "_hip_qref"] = Selec_of_vector(leg + "_hip_qref")
        state[leg + "_hip_qref"].selec(i, i + 1)
        plug(scaled_slider.sout, state[leg + "_hip_qref"].sin)

        # Then we define the reference for the knee joint. We want the knee to move
        # twice as much as the hip and on the opposite direction
        state[leg + "_knee_qref"] = Multiply_double_vector(leg + "_knee_qref")

        state[leg + "_knee_qref"].sin1.value = -2.0
        plug(state[leg + "_hip_qref"].sout, state[leg + "_knee_qref"].sin2)

        # now we need to stack the signals 2 by 2:
        state[leg + "_qref"] = Stack_of_vector(leg + "_qref")
        state[leg + "_qref"].selec1(0, 1)
        state[leg + "_qref"].selec2(0, 1)
        # first element is the hip
        plug(state[leg + "_hip_qref"].sout, state[leg + "_qref"].sin1)
        # second element is the knee
        plug(state[leg + "_knee_qref"].sout, state[leg + "_qref"].sin2)

    robot_state_front_legs = Stack_of_vector("front_legs_state")
    plug(state["fr_qref"].sout, robot_state_front_legs.sin1)
    plug(state["fl_qref"].sout, robot_state_front_legs.sin2)

    robot_state_back_legs = Stack_of_vector("hind_legs_state")
    plug(state["hr_qref"].sout, robot_state_back_legs.sin1)
    plug(state["hl_qref"].sout, robot_state_back_legs.sin2)

    robot_state = Stack_of_vector("robot_des_state")
    plug(robot_state_front_legs.sout, robot_state.sin1)
    plug(robot_state_back_legs.sout, robot_state.sin2)


def draw_simple_graph():
    writeGraph("/tmp/robot_state_reference_from_slider.dot")


print("Creating graph...")
create_simple_graph()
print("Graph loaded.")
print("Dumping graph structure in a file...")
draw_simple_graph()
print("Graph saved.")
