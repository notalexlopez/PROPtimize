
import openmdao.api as om

class ElectronicSpeedController(om.ExplicitComponent):

    def initialize(self):
        self.options.declare('a', default = 1.6054, desc = 'a coefficient for efficiency(throttle) equation: efficiency = a * (1 - 1 / (1 + c*throttle^d))')
        self.options.declare('b', default = 1.6519, desc = 'b coefficient for efficiency(throttle) equation: efficiency = a * (1 - 1 / (1 + c*throttle^d))')
        self.options.declare('c', default = 0.6455, desc = 'c coefficient for efficiency(throttle) equation: efficiency = a * (1 - 1 / (1 + c*throttle^d))')
        self.options.declare("flight_conds", default = 3, desc= "Number of Flight Conditions to Analyze")
        self.options.declare("flight_missions", default = 2, desc = "Number of Flight Missions ot Analyze")

    def setup(self):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        self.add_input('esc_current_in', shape = (fc,fm),units = 'A')
        self.add_input('esc_voltage_in', shape = (fc,fm), units = 'V')
        self.add_input("throttle", shape=(fc,fm), desc="takeoff throttle setting")

        self.add_output('esc_efficiency', shape = (fc,fm))
        self.add_output('esc_voltage_out', shape =(fc,fm), units = 'V')
        self.add_output('esc_current_out', shape = (fc,fm),units = 'A')
        self.add_output('esc_power', shape = (fc,fm), units = 'W')

        self.declare_partials('*', '*', method = 'cs')

    def compute(self, inputs, outputs):
        
        a = self.options['a']
        b = self.options['b']
        c = self.options['c']
        outputs['esc_efficiency'] = a * (1 - 1 / (1 + b*inputs['throttle']**c))

        outputs['esc_voltage_out'] = inputs['esc_voltage_in'] * inputs['throttle'] * outputs['esc_efficiency']
        outputs['esc_current_out'] = inputs['esc_current_in'] / inputs['throttle']
        outputs['esc_power'] = (outputs['esc_efficiency'] - 1) * inputs['esc_current_in'] * inputs['esc_voltage_in']