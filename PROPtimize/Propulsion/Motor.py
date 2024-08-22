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
        self.add_input('motor_mass', units = 'kg' )
        self.add_input('motor_voltage_in', shape = (fc,fm), units = 'V')
        self.add_input('motor_current', shape = (fc,fm), units = 'A')

        self.add_output('rpm', shape = (fc,fm), units = 'rev/min')
        self.add_output('motor_power', shape = (fc,fm), units = 'W')
        self.add_output('motor_kv', units = 'rpm / V')
        self.add_output('motor_resistance', units = 'ohm')


        self.declare_partials('*', '*', method = 'cs')
        

    def compute(self, inputs, outputs):

        outputs['motor_kv'] = (1.3132 * 120) / (inputs['motor_mass'] + 0.01) # FOR SCORPION MOTORS
        outputs['motor_resistance'] = (0.0467 * inputs['motor_idle_current'] ** -1.892)

        voltage_prop = inputs['motor_voltage_in'] - (inputs['motor_current'] * outputs['motor_resistance'])
        outputs['rpm'] = outputs['motor_kv'] * voltage_prop 
        outputs['motor_power'] = -inputs['motor_current']**2 * outputs['motor_resistance'] - inputs['motor_idle_current'] * voltage_prop