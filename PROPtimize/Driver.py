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

prob.model.add_design_var("battery_mass", units = 'kg', lower = .1, upper = 1)
prob.model.add_design_var('motor_idle_current', units = 'A', lower = 1, upper = 3.6)
prob.model.add_design_var('motor_mass', lower = 0.288, upper = 1.701, units = 'kg')
prob.model.add_design_var("D_prop", lower=12, upper=23, units="inch")
prob.model.add_design_var("pitch", lower=3, upper=15, units="deg")
prob.model.add_design_var("throttle", lower = 0.1, upper = 1)
prob.model.add_design_var("motor_peak_current", lower = 1, upper = 225, units = "A") # Limits are based on available motors
prob.model.add_design_var("velocity", units = "m/s", lower =0, upper = 35)

prob.model.add_objective('prop_thrust', scaler = -1)
prob.model.add_constraint("RPM_con", upper = 0)
prob.model.add_constraint("current_con", upper = 0, units = "A")

prob.model_options['*'] = {'flight_conds': 1, "flight_missions": 1, 'props': 1}
prob.setup(check=True)

prob.set_val("num_motors", 1)
prob.set_val("battery_voltage_supply", 22.2, units = "V")
prob.set_val("battery_resistance", 0.05, units = "ohm")

prob.set_val('motor_peak_current', 120, units = "A")
prob.set_val("D_prop", 16, units = "inch")
prob.set_val("pitch", 12, units= "deg")
prob.set_val("throttle", 0.8)
prob.set_val("velocity",  45, units = "ft/s")
prob.set_val('motor_idle_current', 0.91, units = 'A')
prob.set_val('motor_mass', 0.288, units = 'kg')
prob.set_val('battery_mass', .707, units = 'lb')

prob.set_solver_print(level=-1)
prob.set_solver_print(level=2, depth=1)

prob.run_driver()

print(f"Performance Information:")

print(f"    Throttle: {prob.get_val('throttle')}")
print(f"    Thrust {prob.get_val('prop_thrust', units = 'N')} N")
print(f"    Velocity: {prob.get_val('velocity', units = 'm/s')} m/s")
print(f"    Motor Current {prob.get_val('motor_current', units = 'A')} A")
print(f"    Battery Current {prob.get_val('battery_current', units = 'A')} A")
print(f"    battery power {prob.get_val('battery_power', units = 'W')} W")
print(f"    esc power {prob.get_val('esc_power', units = 'W')} W")
print(f"    motor power {prob.get_val('motor_power', units = 'W')} W")
print(f"    power {prob.get_val('prop_power', units = 'W')} W")
print(f"    RPM: {prob.get_val('rpm', units = 'rpm') } rpm")

print(f"\nPropeller Information:")

print(f"    Propeller Diameter: {prob.get_val('D_prop', units='inch')} in")
print(f"    Propeller Pitch: {prob.get_val('pitch', units='deg')} degrees")


print(f"\nMotor Information:")

print(f"    KV{prob.get_val('motor_kv', units = 'rpm/V')} rpm/V")
print(f"    Idle Current{prob.get_val('motor_idle_current', units = 'A')} A")
print(f"    Peak Current{prob.get_val('motor_peak_current', units = 'A')} A")
print(f"    Mass{prob.get_val('motor_mass', units = 'kg')} kg")
print(f"    Resistance: {prob.get_val('motor_resistance', units ='ohm')}")


print(f"\nBattery Information:")

print(f"    Nominal capacity: {prob.get_val('nominal_capacity', units = 'A*h')}")
print(f"    Current {prob.get_val('battery_current', units = 'A')} A")

#TODO: I think there is a local minima at 0 thrust, fixed when adding max current constraint but who knows

# import niceplots #type:ignore
# import matplotlib.pyplot as plt

# plt.style.use(niceplots.get_style("james-light"))
# colors = niceplots.get_colors()

# advance_ratio = 
# fig, ax = plt.subplots()
# (ct_j, ) = plt.plot()