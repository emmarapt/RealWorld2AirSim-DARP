# import random
# import math
# from nodesPlacementOptimization.Transformations import Transformations
#
#
# class SimulatedAnnealing(object):
#
# 	def __init__(self):
#
# 		self.optimizationIndexMax = 0
# 		self.optimizationIndexCurrent = 0
#
# 	def getOptimalTheta(self):
# 		return self.optimalTheta
#
# 	def getOptimalShiftX(self):
# 		return self.optimalShiftX
#
# 	def getOptimalShiftY(self):
# 		return self.optimalShiftY
#
# 	def run(self, cart, cartObst, scanDist):
# 		self.cart = cart
# 		self.cartObst = cartObst
# 		self.scanDist = scanDist
#
# 		# Initial and final temperature
# 		T = 1000
# 		# Temperature at which iteration terminates
# 		Tmin = 5
# 		# Decrease in temperature
# 		alpha = 0.9
# 		# Number of iterations of annealing before decreasing temperature
# 		numIterations = 500
#
# 		# Random initial solution
# 		currentSol = Solution(self.cart, self.cartObst, self.scanDist)
# 		currentSol.Initialization()
# 		currentSol.CreateNew()
#
# 		# Continues annealing until reaching minimum temperature
# 		while T > Tmin:
#
# 			for i in range(numIterations):
#
# 				self.optimizationIndexCurrent = currentSol.getOptimizationIndex()
#
# 				# Reassigns global minimum accordingly
# 				if self.optimizationIndexCurrent > self.optimizationIndexMax:
# 					self.optimizationIndexMax = self.optimizationIndexCurrent
# 					self.optimalTheta = currentSol.getTheta()
# 					self.optimalShiftX = currentSol.getShiftX()
# 					self.optimalShiftY = currentSol.getShiftY()
#
# 				newSol = Solution(self.cart, self.cartObst, self.scanDist)
# 				newSol.Random()
#
# 				newSol.CreateNew()
# 				ap = math.pow(math.e, (self.optimizationIndexCurrent - newSol.getOptimizationIndex()) / T)
#
# 				if ap > random.random():
# 					currentSol = newSol
#
# 			T = T * alpha  # Decreases T, cooling phase
# 			# print("Change number of iteration i to i = 500")
# 		print("\n ~~~ Final value of optimization index: ", self.optimizationIndexMax, " ~~~ ")
#
#
# class Solution(object):
#
# 	def __init__(self, cart, cartObst, scanDist):
#
# 		self.cart = cart
# 		self.cartObst = cartObst
# 		self.scanDist = scanDist
#
# 	def Initialization(self):
# 		self.theta = 0
# 		self.shiftX = self.scanDist
# 		self.shiftY = self.scanDist
#
# 	def Random(self):
# 		self.theta = float(random.random()) * 90
# 		self.shiftX = float(random.random()) * 2 * self.scanDist
# 		self.shiftY = float(random.random()) * 2 * self.scanDist
#
# 	def Neighbor(self):
#
# 		if random.random() > .5:
# 			a = 45
# 		else:
# 			a = -45
# 		if random.random() > .5:
# 			b = self.scanDist / 4
# 		else:
# 			b = -self.scanDist / 4
#
# 		if random.random() > .5:
# 			c = self.scanDist / 4
# 		else:
# 			c = -self.scanDist / 4
#
# 		self.theta = self.theta + a * (float(random.random()))
# 		self.shiftX = self.shiftX + b * (float(random.random()))
# 		self.shiftY = self.shiftY + c * (float(random.random()))
#
# 	def CreateNew(self):
#
# 		randomSol = Transformations()
#
# 		randomSol.setTheta(self.theta)
# 		randomSol.setShiftX(self.shiftX)
# 		randomSol.setShiftY(self.shiftY)
# 		self.optimizationIndex = randomSol.rotateAndShift(self.cart, self.cartObst, self.scanDist)
#
# 	def getTheta(self):
# 		return self.theta
#
# 	def getShiftX(self):
# 		return self.shiftX
#
# 	def getShiftY(self):
# 		return self.shiftY
#
# 	def getOptimizationIndex(self):
# 		return self.optimizationIndex

import random
import math
from RealWorld.nodesPlacementOptimization.Transformations import Transformations
from scipy.optimize import basinhopping
from scipy.optimize import dual_annealing
import numpy as np


class SimulatedAnnealing(object):

    def __init__(self):
        self.optimizationIndexMax = 0
        self.optimizationIndexCurrent = 0

    def getOptimalTheta(self):
        return self.optimalTheta

    def getOptimalShiftX(self):
        return self.optimalShiftX

    def getOptimalShiftY(self):
        return self.optimalShiftY

    def run(self, cart, cartObst, scanDist):
        self.cart = cart
        self.cartObst = cartObst
        self.scanDist = scanDist

        # Initial and final temperature
        T = 1000
        # Temperature at which iteration terminates
        Tmin = 5
        # Decrease in temperature
        alpha = 0.9
        # Number of iterations of annealing before decreasing temperature
        numIterations = 500  # TODO: Savvas in java has 500 iterations

        # Random initial solution
        # currentSol = Solution(self.cart, self.cartObst, self.scanDist)
        # currentSol.Initialization()
        # currentSol.CreateNew()

        # Continues annealing until reaching minimum temperature
        # while T > Tmin:

        # 	for i in range(numIterations):

        # 		self.optimizationIndexCurrent = currentSol.getOptimizationIndex()

        # 		# Reassigns global minimum accordingly
        # 		if self.optimizationIndexCurrent > self.optimizationIndexMax:
        # 			self.optimizationIndexMax = self.optimizationIndexCurrent
        # 			self.optimalTheta = currentSol.getTheta()
        # 			self.optimalShiftX = currentSol.getShiftX()
        # 			self.optimalShiftY = currentSol.getShiftY()

        # 		newSol = Solution(self.cart, self.cartObst, self.scanDist)
        # 		newSol.Random()

        # 		newSol.CreateNew()
        # 		ap = math.pow(math.e, (self.optimizationIndexCurrent - newSol.getOptimizationIndex()) / T)

        # 		if ap > random.random():
        # 			currentSol = newSol

        # 	T = T * alpha  # Decreases T, cooling phase
        # # print("Change number of iteration i to i = 500")
        theta_bounds = (0, 90)
        x_bounds = (0, 2 * self.scanDist)
        y_bounds = (0, 2 * self.scanDist)
        bounds = [theta_bounds, x_bounds, y_bounds]
        mytakestep = MyTakeStep(scanDist=self.scanDist)
        theta = float(random.random()) * 90
        shiftX = float(random.random()) * 2 * self.scanDist
        shiftY = float(random.random()) * 2 * self.scanDist
        minimizer_kwargs = {"args": (self.cart, self.cartObst, self.scanDist), "bounds": bounds}
        # ret = basinhopping(CreateNew, [theta,shiftX,shiftY], minimizer_kwargs=minimizer_kwargs,stepsize=0.5,take_step=mytakestep,T=1000, niter=500)
        ret = dual_annealing(CreateNew, bounds, args=(minimizer_kwargs['args']), maxiter=500)
        # print(ret)
        self.optimalTheta, self.optimalShiftX, self.optimalShiftY = ret['x']

        print("\n ~~~ Final value of optimization index: ",
              -1 * CreateNew(ret['x'], self.cart, self.cartObst, self.scanDist), " ~~~ ")


def CreateNew(x, cart, cartObst, scanDist):
    randomSol = Transformations()

    randomSol.setTheta(x[0])
    randomSol.setShiftX(x[1])
    randomSol.setShiftY(x[2])
    # print(randomSol.rotateAndShift(cart, cartObst, scanDist))
    return -randomSol.rotateAndShift(cart, cartObst, scanDist)


class MyTakeStep:

    def __init__(self, scanDist, stepsize=0.5):
        self.stepsize = stepsize
        self.scan = scanDist

        self.rng = np.random.default_rng()

    def __call__(self, x):
        s = self.stepsize

        x[0] += self.rng.uniform(-45 * s, 45 * s)
        x[1] += self.rng.uniform(-2 * self.scan * s, 2 * self.scan * s)
        x[2] += self.rng.uniform(-2 * self.scan * s, 2 * self.scan * s)

        return x

# class Solution(object):

# 	def __init__(self, cart, cartObst, scanDist):

# 		self.cart = cart
# 		self.cartObst = cartObst
# 		self.scanDist = scanDist

# 	def Initialization(self):
# 		self.theta = 0
# 		self.shiftX = self.scanDist
# 		self.shiftY = self.scanDist

# 	def Random(self):
# 		self.theta = float(random.random()) * 90
# 		self.shiftX = float(random.random()) * 2 * self.scanDist
# 		self.shiftY = float(random.random()) * 2 * self.scanDist

# 	def Neighbor(self):

# 		if random.random() > .5:
# 			a = 45
# 		else:
# 			a = -45
# 		if random.random() > .5:
# 			b = self.scanDist / 4
# 		else:
# 			b = -self.scanDist / 4

# 		if random.random() > .5:
# 			c = self.scanDist / 4
# 		else:
# 			c = -self.scanDist / 4

# 		self.theta = self.theta + a * (float(random.random()))
# 		self.shiftX = self.shiftX + b * (float(random.random()))
# 		self.shiftY = self.shiftY + c * (float(random.random()))

# 	def CreateNew(self):

# 		randomSol = Transformations()

# 		randomSol.setTheta(self.theta)
# 		randomSol.setShiftX(self.shiftX)
# 		randomSol.setShiftY(self.shiftY)
# 		self.optimizationIndex = randomSol.rotateAndShift(self.cart, self.cartObst, self.scanDist)

# 	def getTheta(self):
# 		return self.theta

# 	def getShiftX(self):
# 		return self.shiftX

# 	def getShiftY(self):
# 		return self.shiftY

# 	def getOptimizationIndex(self):
# 		return self.optimizationIndex
