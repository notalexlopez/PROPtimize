import openmdao.api as om
import numpy as np
from Propulsion.Propeller import Propeller
from Propulsion.RPMConstraints import RPMConstraints
from Propulsion.Battery import Battery
from Propulsion.Electronics import ElectronicSpeedController
from Propulsion.Motor import Motor
from Propulsion.PowerResiduals import PowerResiduals
from Propulsion.PropCoefficients import PropCoefficients


class PropulsionGroup(om.Group):
    """[summary]

    Inputs:
        D_prop: propeller diameter, length
        pitch: propeller pitch, angle

    Outputs:
        Thrust
    """

    def setup(self):
        
        self.add_subsystem(
            'battery', 
            Battery(),
            promotes_inputs= [
                'battery_voltage_supply', 
                'battery_mass',
                'battery_resistance',
                'battery_current'
            ], 
            promotes_outputs= [
                'battery_energy',
                'battery_voltage_out',
                'battery_power',
                'nominal_capacity'
            ]
        )
        self.add_subsystem(
            'esc', 
            ElectronicSpeedController(),
            promotes_inputs= [
                'esc_voltage_in',
                'esc_current_in',
                'throttle',
            ], 
            promotes_outputs= [
                'esc_efficiency',
                'esc_voltage_out',
                'esc_current_out',
                'esc_power',
            ]
        )
        self.add_subsystem(
            'motor', 
            Motor(),
            promotes_inputs= [
                'motor_idle_current',
                'motor_mass',
                'motor_voltage_in',
                'motor_current',
            ], 
            promotes_outputs= [
                'rpm',
                'motor_power',
                'motor_kv',
                'motor_resistance'
            ]
        )
        
        self.add_subsystem(
            "PropCoefficients",
            PropCoefficients(),
            promotes_inputs=[
                "D_prop",
                "pitch",
                "rpm",
                "velocity",
            ],
            promotes_outputs=[
                "ct",
                "cp"
            ],
        )

        self.add_subsystem(
            "Propeller",
            Propeller(),
            promotes_inputs=[
                "rho",
                "D_prop",
                "rpm",
                "ct",
                "cp",
                "num_motors",
            ],
            promotes_outputs=[
                "prop_thrust",
                "prop_power"
            ],
        )

        self.add_subsystem(
            "RPMConstraints",
            RPMConstraints(),
            promotes_inputs=["rpm", "D_prop"],
            promotes_outputs=["RPM_con"],
        )
        self.add_subsystem(
            'power_net', 
            PowerResiduals(), 
            promotes_inputs = [
                'battery_power', 
                'esc_power',
                'motor_power',
                'prop_power',
            ],
            promotes_outputs= ['res_current']
        )
        
        self.connect('battery_voltage_out', 'esc_voltage_in')
        self.connect('esc_voltage_out', 'motor_voltage_in')
        self.connect('esc_current_out', 'motor_current')
        self.connect('res_current', ['battery_current', 'esc_current_in'])