# import openmdao.api as om
# from Model import PropModel
# import numpy as np
# import matplotlib.pyplot as plt
# import niceplots

# prob = om.Problem()
# prob.model = PropModel()
# # Driver setup
# prob.driver = om.pyOptSparseDriver()
# prob.driver.options["optimizer"] = "IPOPT"
# prob.driver.options["debug_print"] = ["desvars", "nl_cons", "objs"]
# recorder = om.SqliteRecorder("RECORDER.sql")
# prob.driver.add_recorder(recorder)
# prob.driver.recording_options["includes"] = ["*"]
# prob.driver.recording_options["record_objectives"] = True
# prob.driver.recording_options["record_constraints"] = True
# prob.driver.recording_options["record_desvars"] = True
# # Set up driver recorder

# prob.model.add_constraint("RPM_con", upper = 0)
# prob.model.add_constraint("current_con", upper = 0, units = "A")

# prob.model_options['*'] = {'flight_conds': 1, "flight_missions": 6, 'props': 1}

# prob.setup(check=True)

# prob.set_val("rho", 1.153570405, units = "kg/m**3")
# prob.set_val("num_motors", 1)
# prob.set_val("battery_voltage_supply", np.array([24.8, 24.7, 24.2, 23.4, 22.5, 21.7]), units="V")
# prob.set_val("battery_resistance", 0.05, units = "ohm")

# prob.set_val("throttle", np.array([ 0.1, 0.3, 0.5, 0.7, 0.9, 1.0]))
# prob.set_val("D_prop", 16, units="inch")
# prob.set_val("pitch", 10, units="deg")
# prob.set_val("velocity", np.zeros(6), units="m/s")
# prob.set_val('motor_idle_current', 0.91, units='A')
# prob.set_val('motor_mass', 0.353, units='kg')
# prob.set_val('battery_mass', 0.707, units='lb')
# prob.set_val('motor_peak_current', 70, units="A")

# prob.set_solver_print(level=2, depth=1)
# prob.run_model()

# print(f"Performance Information:")

# print(f"    Throttle: {prob.get_val('throttle')}")
# print(f"    Thrust {prob.get_val('prop_thrust', units = 'N')} N")
# print(f"    Velocity: {prob.get_val('velocity', units = 'm/s')} m/s")
# print(f"    Motor Current {prob.get_val('motor_current', units = 'A')} A")
# print(f"    Battery Current {prob.get_val('battery_current', units = 'A')} A")
# print(f"    battery power {prob.get_val('battery_power', units = 'W')} W")
# print(f"    esc power {prob.get_val('esc_power', units = 'W')} W")
# print(f"    motor power {prob.get_val('motor_power', units = 'W')} W")
# print(f"    power {prob.get_val('prop_power', units = 'W')} W")
# print(f"    RPM: {prob.get_val('rpm', units = 'rpm') } rpm")

# print(f"\nPropeller Information:")

# print(f"    Propeller Diameter: {prob.get_val('D_prop', units='inch')} in")
# print(f"    Propeller Pitch: {prob.get_val('pitch', units='deg')} degrees")


# print(f"\nMotor Information:")

# print(f"    KV{prob.get_val('motor_kv', units = 'rpm/V')} rpm/V")
# print(f"    Idle Current{prob.get_val('motor_idle_current', units = 'A')} A")
# print(f"    Peak Current{prob.get_val('motor_peak_current', units = 'A')} A")
# print(f"    Mass{prob.get_val('motor_mass', units = 'kg')} kg")
# print(f"    Resistance: {prob.get_val('motor_resistance', units ='ohm')}")


# print(f"\nBattery Information:")

# print(f"    Nominal capacity: {prob.get_val('nominal_capacity', units = 'A*h')}")
# print(f"    Current {prob.get_val('battery_current', units = 'A')} A")

# plt.style.use(niceplots.get_style("james-dark"))
# colors = niceplots.get_colors()

# p_test = np.array([1.92, 41.62, 211.95, 479.77, 797.126, 1051.8])
# t_test = np.array([0.62, 5.71, 17.7, 30, 41.2, 46.8])
# fig, ax = plt.subplots()
# print(prob.get_val('rho', units = 'kg/m**3'))
# (propt, ) = ax.plot(prob.get_val('prop_power', units = 'W')[0], prob.get_val('prop_thrust', units = 'N')[0], label = 'PROPtimize')
# (test, ) = ax.plot(p_test, t_test, "o", label = "Static Tests 1/28")

# ax.set_xlabel("Power")
# ax.set_ylabel("Thrust")
# niceplots.label_line_ends(ax)
# niceplots.adjust_spines(ax)
# niceplots.save_figs(fig, "verification16x10", 'pdf')