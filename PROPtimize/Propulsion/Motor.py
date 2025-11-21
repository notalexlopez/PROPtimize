import numpy as np
import openmdao.api as om

class Motor(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("flight_conds", default = 3, desc= "Number of Flight Conditions to Analyze")
        self.options.declare("flight_missions", default = 2, desc = "Number of Flight Missions ot Analyze")

    def setup(self):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        self.add_input('motor_idle_current', units = 'A')
        self.add_input('max_cont_current', units = "A")
        self.add_input('motor_voltage_in', shape = (fc,fm), units = 'V')
        self.add_input('motor_current', shape = (fc,fm), units = 'A')
        self.add_input('motor_kv', units = 'rpm / V')

        self.add_output('rpm', shape = (fc,fm), units = 'rev/min')
        self.add_output('motor_power', shape = (fc ,fm), units = 'W')
        self.add_output('motor_mass', units = 'kg')
        self.add_output('motor_resistance', units = 'ohm')
        self.add_output('current_con', shape = (fc, fm), units = "A")


        self.declare_partials('*', '*', method = 'cs')
        

    def compute(self, inputs, outputs):

        outputs['motor_mass'] = (2.10553674 * inputs['max_cont_current'] / inputs['motor_kv']) + -0.08083469729

        voltage_prop = inputs['motor_voltage_in'] - (inputs['motor_current'] * outputs['motor_resistance'])
        outputs['rpm'] = inputs['motor_kv'] * voltage_prop 
        outputs['motor_power'] = -inputs['motor_current']**2 * outputs['motor_resistance'] - inputs['motor_idle_current'] * voltage_prop

        outputs["current_con"] = inputs["motor_current"] - inputs["max_cont_current"]