import numpy as np
import openmdao.api as om

#Might be a 3x2, may be screwed here. 
class PowerResiduals(om.ImplicitComponent):
    
    def initialize(self):
        self.options.declare("flight_conds", default = 3, desc= "Number of Flight Conditions to Analyze")
        self.options.declare("flight_missions", default = 2, desc = "Number of Flight Missions ot Analyze")

    def setup(self):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        self.add_input('battery_power', shape = (fc,fm), units = 'W')
        self.add_input('esc_power', shape = (fc,fm), units = 'W')
        self.add_input('motor_power', shape = (fc,fm), units = 'W')
        self.add_input('prop_power', shape = (fc,fm), units = 'W')

        self.add_output('res_current', shape =(fc,fm), units = 'A', val = 30)
        self.add_residual('power_net', shape = (fc, fm), units = 'W')

        self.declare_partials('*', '*', method = 'cs')

    def apply_nonlinear(self, inputs, outputs, residuals):
        residuals['power_net'] = inputs['battery_power'] + inputs['esc_power'] + inputs['motor_power'] - inputs['prop_power']