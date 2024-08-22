import openmdao.api as om
from Propulsion.PropulsionGroup import PropulsionGroup

class PropModel(om.Group):
    def initialize(self):
        self.options.declare("flight_conds", default = 1, desc= "Number of Flight Conditions to Analyze")
        self.options.declare("flight_missions", default = 1, desc = "Number of Flight Missions ot Analyze")
        self.options.declare("props", default = 1, desc="Number of Props to optimize, should be no more than fm")
        
    def setup(self):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        p = self.options["props"]

        self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)
        self.nonlinear_solver.options["maxiter"] = 30
        self.nonlinear_solver.options["err_on_non_converge"] = False
        self.nonlinear_solver.linesearch = om.BoundsEnforceLS()
        self.nonlinear_solver.linesearch.options["bound_enforcement"] = "scalar"
        self.nonlinear_solver.linesearch.options["print_bound_enforce"] = True
        self.linear_solver = om.DirectSolver(assemble_jac=True)#, rhs_checking =True)

        indeps = self.add_subsystem(
            "indeps",
            om.IndepVarComp(),
            promotes_outputs=[
                "D_prop",
                "pitch",
                "throttle",
                "velocity",
            ],
        )
        indeps.add_output("D_prop", units="inch", shape = p, desc="propeller diameter")
        indeps.add_output("pitch", units="deg", shape = p,  desc="propeller pitch")
        indeps.add_output("throttle", shape = (fc,fm), desc = "throttle setting")
        indeps.add_output("velocity", shape = (fc,fm), desc = "optimized velocities", units = "m/s")

        self.add_subsystem(
            "PropulsionGroup",
            PropulsionGroup(),
            promotes_inputs=["*"],
            promotes_outputs=["*"],
        )
        