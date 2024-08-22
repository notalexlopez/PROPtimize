import openmdao.api as om
import numpy as np 


class Propeller(om.ExplicitComponent):

    def initialize(self):
        self.options.declare("flight_conds", default = 3, desc= "Number of Flight Conditions to Analyze")
        self.options.declare("flight_missions", default = 2, desc = "Number of Flight Missions ot Analyze")
        self.options.declare("props", default = 1, desc="Number of Props to optimize, should be no more than fm")

    def setup(self):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        p = self.options["props"]
        self.add_input("rho", units="kg/m**3", desc="air density")
        self.add_input("D_prop", units="m", shape = p, desc="propeller diameter")
        self.add_input("rpm", shape = (fc,fm), units = "rev/s")
        self.add_input("ct", shape = (fc,fm))
        self.add_input("cp", shape = (fc,fm))
        self.add_input("num_motors", desc="number of motors")
        self.add_output("prop_thrust", shape =(fc,fm), units ="N")
        self.add_output("prop_power", shape = (fc,fm), units = "W")

        self.declare_partials("*", "*", method="cs")


    def compute(self, inputs, outputs):
        rho = inputs["rho"]
        D = inputs["D_prop"]
        n = inputs ["rpm"]

        outputs["prop_thrust"] = (rho * n**2 * D**4 * inputs["ct"] * inputs["num_motors"])
        outputs["prop_power"] = (rho * n**3 * D**5 * inputs["cp"])