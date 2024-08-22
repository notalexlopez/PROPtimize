import numpy as np
import openmdao.api as om

#&& CHECK NECESSARY MODS FOR FLIGHT CONDS
#CONDITION SHOULD BE BASED ON CURRENT, BATTERY WILL PROVIDE VARYING CURRENT FOR EACH CONDITION BASED ON ENDURANCE REQUIREMENTS
class Battery(om.ExplicitComponent):

    def initialize(self):
        self.options.declare("flight_conds", default = 3, desc= "Number of Flight Conditions to Analyze")
        self.options.declare("flight_missions", default = 2, desc = "Number of Flight Missions ot Analyze")

    def setup(self):

        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]

        self.add_input('battery_voltage_supply', shape = fm, units = "V") #Running under the assumption that one can replace batteries
        self.add_input('battery_mass', shape = fm, units = "kg")
        self.add_input('battery_resistance', shape = fm, units = 'ohm') 
        self.add_input('battery_current', shape = (fc,fm), units = 'A')

        self.add_output('battery_voltage_out', shape = (fc,fm), units = 'V')
        self.add_output('battery_power', shape = (fc,fm), units = 'W')
        self.add_output('nominal_capacity', shape = fm, units = 'A*h')
        self.add_output('battery_energy', shape = fm, units = "W*h",  desc ="individual battery energy")
        

        self.declare_partials('*', '*', method = 'cs')
        

    def compute(self, inputs, outputs):
        outputs['nominal_capacity'] = inputs['battery_mass'] * 7.3 - 0.246
        outputs['battery_energy']= inputs['battery_voltage_supply'] * outputs['nominal_capacity']

        outputs['battery_voltage_out'] = (inputs['battery_voltage_supply'] - inputs['battery_current'] * inputs['battery_resistance'])
        outputs['battery_power'] =  (inputs['battery_current'] * inputs['battery_voltage_supply'] - inputs['battery_current']**2 * inputs['battery_resistance'])
        