import openmdao.api as om
import numpy as np


class RPMConstraints(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("flight_conds", default = 3, desc= "Number of Flight Conditions to Analyze")
        self.options.declare("flight_missions", default = 2, desc = "Number of Flight Missions ot Analyze")
        self.options.declare("props", default = 1, desc="Number of Props to optimize, should be no more than fm")
        
    def setup(self):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        p = self.options["props"]
        self.add_input(
            "rpm", shape = (fc,fm), units = "rev/min")
        self.add_input("D_prop", units="inch", shape = p, desc="propeller diameter")

        self.add_output("RPM_con", shape=(fc,fm), desc="RPM limit")

        self.declare_partials("*", "*", method="cs")

    def compute(self, inputs, outputs):
        outputs["RPM_con"] = inputs["rpm"] - 150000 / inputs["D_prop"]
