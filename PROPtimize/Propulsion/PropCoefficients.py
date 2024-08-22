# Written by Benjamin Levy
# Re-written by Alejandro Lopez 

import openmdao.api as om
import numpy as np
import pickle

from smt.surrogate_models import KPLSK #type:ignore

class PropCoefficients(om.ExplicitComponent):
    """Encapsulated surrogate model to compute thrust and power
    coefficients from prop dimensions and flight conditions
    using Surrogat Modeling Toolbox
    https://smt.readthedocs.io/en/latest/index.html

    """

    def initialize(self):
        self.options.declare("flight_conds", default = 3, desc= "Number of Flight Conditions to Analyze")
        self.options.declare("flight_missions", default = 2, desc = "Number of Flight Missions ot Analyze")
        self.options.declare("props", default = 1, desc="Number of Props to optimize, should be no more than fm")

        """Initializing surrogate model, reading and sampling data"""
        '''
        self.thrust_sm = KPLSK(n_comp=4, eval_noise=True, print_global=False)
        self.power_sm = KPLSK(n_comp=4, eval_noise=True, print_global=False)

        x, ct, cp = PropDataReader()
        x, ct, cp, xVal, ctVal, cpVal = SampleDataLHScosine(x, ct, cp)

        xlim = np.array(
            [
                 [np.min(x[:, 0]), np.max(x[:, 0]) + 1],
                 [np.min(x[:, 1]), np.max(x[:, 1]) + 1],
                 [np.min(x[:, 2]), np.max(x[:, 2]) + 1],
                 [np.min(x[:, 3]), np.max(x[:, 3]) + 1],
            ]
        )

        self.thrust_sm.set_training_values(x, ct)
        self.thrust_sm.train()
        self.power_sm.set_training_values(x, cp)
        self.power_sm.train()

        with open("PickledSurrogateModels/power_sm.pkl", "wb") as fp:
            pickle.dump(self.power_sm, fp)
        with open("PickledSurrogateModels/thrust_sm.pkl", "wb") as fp:
            pickle.dump(self.thrust_sm, fp)
        '''

        with open(
            "PickledSurrogateModels/power_sm.pkl", #Now using 1.3.0
            "rb",
        ) as fp:
            self.power_sm = pickle.load(fp) 
        with open(
            "PickledSurrogateModels/thrust_sm.pkl", #Now using 1.3.0
            "rb",
        ) as fp:
            self.thrust_sm = pickle.load(fp)
            #'''

    def setup(self):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        p=self.options["props"]
        self.add_input("D_prop", units="m", shape = p, desc="propeller diameter")
        self.add_input("pitch", units="deg", shape = p, desc="propeller pitch")

        self.add_input('rpm', shape = (fc,fm), units = "rev/s")
        self.add_input('velocity', shape = (fc,fm), units = "m/s")
        self.add_output('ct', shape = (fc,fm),desc = "thrust coefficients")
        self.add_output('cp', shape = (fc,fm),desc="power coefficients")

        self.declare_partials('*', '*')

    def compute_partials(self, inputs, partials):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        p=self.options["props"]
        D = inputs["D_prop"]
        pitch = inputs["pitch"]
        n = inputs['rpm']
        V = inputs['velocity']
        #Create empty array for each condition depending on how many are declared
        #Fill the conditiosn with the amount of flight missions that have been declared
        x=0
        cond = []
        #For the diameter and pitch, when adding multiprop, will be D[y]
        if p > 1:
            while x < fc:
                y= 0
                while y < fm:
                    temps = np.ndarray((0,4))
                    temps = [np.vstack((temps, np.array([D[y], pitch[y], n[x][y], V[x][y]])))]
                    cond.append(temps)
                    y+=1
                x+=1
        if p == 1: 
            while x < fc:
                    y= 0
                    while y < fm:
                        temps = np.ndarray((0,4))
                        temps = [np.vstack((temps, np.array([D[0], pitch[0], n[x][y], V[x][y]])))]
                        cond.append(temps)
                        y+=1
                    x+=1

        #Predict the partials for each condition with respect to each (First declare range of conditions)
        ranger = int(np.size(cond)/4)

        #Diameter
        tdzeros = np.zeros((fc * fm , p))
        td = []
        for z in range(ranger):
            appender=self.thrust_sm.predict_derivatives(cond[z][0], 0)
            td.append(appender)
        for m in range(p):
            for r in range(m,fc * fm, p):
                tdzeros[r][m]= td[r]
        partials["ct", "D_prop"] = tdzeros

        pdzeros = np.zeros((fc * fm , p))
        pd = []
        for z in range(ranger):
            appender=self.power_sm.predict_derivatives(cond[z][0], 0)
            pd.append(appender)
        for m in range(p):
            for r in range(m,fc * fm, p):
                pdzeros[r][m]= pd[r]
        partials["cp", "D_prop"] = pdzeros

        #Pitch 
        tpzeros = np.zeros((fc * fm , p))
        tp = []
        for z in range(ranger):
            appender=self.thrust_sm.predict_derivatives(cond[z][0], 1)
            tp.append(appender)
        for m in range(p):
            for r in range(m,fc * fm, p):
                tpzeros[r][m]= tp[r]
        partials["ct", "pitch"] = tpzeros

        ppzeros = np.zeros((fc * fm ,  p))
        pp = []
        for z in range(ranger):
            appender=self.power_sm.predict_derivatives(cond[z][0], 1)
            pp.append(appender)
        for m in range(p):
            for r in range(m,fc * fm, p):
                ppzeros[r][m]= pp[r]
        partials["cp", "pitch"] = ppzeros

        #RPM
        tnzeros = np.zeros((fc * fm , fc * fm))
        tn = []
        for z in range(ranger):
            appender=self.thrust_sm.predict_derivatives(cond[z][0], 2)
            tn.append(appender)
        np.fill_diagonal(
            tnzeros,
            tn
        )
        partials["ct", "rpm"] = tnzeros

        pnzeros = np.zeros((fc * fm , fc * fm ))
        pn = []
        for z in range(ranger):
            appender= self.power_sm.predict_derivatives(cond[z][0], 2)
            pn.append(appender)
        np.fill_diagonal(
            pnzeros,
            pn
        )
        partials["cp", "rpm"] = pnzeros

        #Velocity
        tvzeros = np.zeros((fc * fm , fc * fm))
        tv = []
        for z in range(ranger):
            appender=self.thrust_sm.predict_derivatives(cond[z][0], 3)
            tv.append(appender)
        np.fill_diagonal(
            tvzeros,
            tv
        )
        partials["ct", "velocity"] = tvzeros
        
        pvzeros = np.zeros((fc * fm , fc * fm ))
        pv = []
        for z in range(ranger):
            appender= self.power_sm.predict_derivatives(cond[z][0], 3)
            pv.append(appender)
        np.fill_diagonal(
            pvzeros,
            pv
        )
        partials["cp", "velocity"] = pvzeros
    
    def compute(self, inputs, outputs):
        fc = self.options["flight_conds"]
        fm = self.options["flight_missions"]
        p = self.options["props"]
        D = inputs["D_prop"]
        pitch = inputs["pitch"]
        n = inputs['rpm']
        V = inputs['velocity']
        #Create empty array for each condition depending on how many are declared
        #Fill the conditiosn with the amount of flight missions that have been declared
        x=0
        cond = []
        #For the diameter and pitch, when adding multiprop, will be D[y], &&
        if p > 1: 
            while x < fc:
                y= 0
                while y < fm:
                    appender = np.array([D[y], pitch[y], n[x][y], V[x][y]])
                    cond.append(appender)
                    y+=1
                x+=1
        if p == 1: 
            while x < fc:
                y= 0
                while y < fm:
                    appender = np.array([D[0], pitch[0], n[x][y], V[x][y]])
                    cond.append(appender)
                    y+=1
                x+=1
        #Unlike in partials, here one stacks all conditions, predicting all at once. 
        topredict = np.vstack(cond)
        
        #test this first, if it works could be crazy
        outputs["ct"] = self.thrust_sm.predict_values(topredict)
        outputs["cp"] = self.power_sm.predict_values(topredict)