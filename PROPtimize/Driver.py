import openmdao.api as om
from Model import PropModel
import numpy as np

prob = om.Problem()

prob.model = PropModel()
# Driver setup
prob.driver = om.pyOptSparseDriver()
prob.driver.options["optimizer"] = "IPOPT"
prob.driver.options["debug_print"] = ["desvars", "nl_cons", "objs"]

recorder = om.SqliteRecorder("RECORDER.sql")
prob.driver.add_recorder(recorder)
prob.driver.recording_options["includes"] = ["*"]
prob.driver.recording_options["record_objectives"] = True
prob.driver.recording_options["record_constraints"] = True
prob.driver.recording_options["record_desvars"] = True
# Set up driver recorder

'''For endurance based optimization, it is recommended to optimize for battery mass, 
otherwise a standard battery mass of 0.71 kg is added.'''

#prob.model.add_design_var("battery_mass", units = 'kg', lower = .1, upper = 1)
prob.model.add_design_var('motor_idle_current', units = 'A', lower = 1, upper = 3.6)
prob.model.add_design_var('motor_mass', lower = 0.288, upper = 1.701, units = 'kg')
prob.model.add_design_var("D_prop", lower=12, upper=23, units="inch")
prob.model.add_design_var("pitch", lower=3, upper=15, units="deg")
#prob.model.add_design_var("throttle", lower = 0.1, upper = 1)
#prob.model.add_design_var("velocity", units = "m/s", lower =1, upper = 35)

prob.model.add_objective('prop_thrust', scaler = -1)
prob.model.add_constraint("RPM_con", upper = 0)

prob.model_options['*'] = {'flight_conds': 1, "flight_missions": 1, 'props': 1}
prob.setup(check=True)

prob.set_val("num_motors", 1)
prob.set_val("battery_voltage_supply", 22.2, units = "V")
prob.set_val("battery_resistance", 0.012, units = "ohm")

prob.set_val("D_prop", 14, units = "inch")
prob.set_val("pitch", 5, units= "deg")
prob.set_val("throttle", 0.8)
prob.set_val("velocity", 45, units = "ft/s")
prob.set_val('motor_idle_current', 2.15, units = 'A')
prob.set_val('motor_mass', 0.5234, units = 'kg')
prob.set_val('battery_mass', .71, units = 'lb')

prob.set_solver_print(level=-1)
prob.set_solver_print(level=2, depth=1)

prob.run_driver()

print(f"Optimal Propeller Diameter: {prob.get_val('D_prop', units='inch')} in")
print(f"Optimal Propeller Pitch: {prob.get_val('pitch', units='deg')} degrees")
print(f"Throttle: {prob.get_val('throttle')}")
print(f"thrust {prob.get_val('prop_thrust', units = 'N')} N")
print(f"battery power {prob.get_val('battery_power', units = 'W')} W")
print(f"esc power {prob.get_val('esc_power', units = 'W')} W")
print(f"motor power {prob.get_val('motor_power', units = 'W')} W")
print(f"power {prob.get_val('prop_power', units = 'W')} W")
print(f"rpm: {prob.get_val('rpm', units = 'rpm') } rpm")
print(f"motor resistance: {prob.get_val('motor_resistance', units ='ohm')}")
print(f"Optimal Motor KV{prob.get_val('motor_kv', units = 'rpm/V')} rpm/V")
print(f"Optimal Motor Idle Current{prob.get_val('motor_idle_current', units = 'A')} A")
print(f"Optimal Motor Mass{prob.get_val('motor_mass', units = 'kg')} kg")
print(f"nominal capacity: {prob.get_val('nominal_capacity', units = 'A*h')}")
print(f"battery current {prob.get_val('battery_current', units = 'A')} A")