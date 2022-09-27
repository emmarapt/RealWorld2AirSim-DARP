import matplotlib.pyplot as plt

""" Path NED Visualization with initial positions"""


class PlotNEDPaths:

	def __init__(self, cart, cartObst, droneNo, missionWaypointsNED, init_posNED, optuna):
		self.cart = cart
		self.cartObst = cartObst
		self.droneNo = droneNo
		self.missionWaypointsNED = missionWaypointsNED
		self.init_posNED = init_posNED
		self.optuna = optuna

	def plot(self):

		fig = plt.figure()
		ax = fig.add_subplot(111)

		polyx = []
		polyy = []
		for i in range(len(self.cart)):

			polyx.append(self.cart[i][1])
			polyy.append(self.cart[i][0])
		polyx.append(self.cart[0][1])
		polyy.append(self.cart[0][0])


		if len(self.cartObst) > 0:
			obstx = []
			obsty = []
			for i in range(len(self.cartObst)):
				for j in range(len(self.cartObst[i])):
					obstx.append(self.cartObst[i][j][1])
					obsty.append(self.cartObst[i][j][0])
			obstx.append(self.cartObst[0][0][1])
			obsty.append(self.cartObst[0][0][0])



		x = [[] for _ in range(self.droneNo)]
		y = [[] for _ in range(self.droneNo)]

		x_init_pos = [[] for _ in range(self.droneNo)]
		y_init_pos = [[] for _ in range(self.droneNo)]

		for i in range(self.droneNo):
			x_init_pos[i].append(self.init_posNED[i][1])
			y_init_pos[i].append(self.init_posNED[i][0])

			#for j in range(len(self.missionWaypointsNED[i][0])):
				#x[i].append(self.missionWaypointsNED[i][0][j][1])
				#y[i].append(self.missionWaypointsNED[i][0][j][0])
			for j in range(len(self.missionWaypointsNED[i])):
				x[i].append(self.missionWaypointsNED[i][j][1])
				y[i].append(self.missionWaypointsNED[i][j][0])

		# plot path for each robot + init_pos
		for i in range(self.droneNo):
			plt.plot(x[i], y[i], label='Drone{}'.format(i + 1))
			# plt.plot(x[i], y[i], label = '', color='steelblue')#'Drone{}'.format(i + 1))
			# plt.scatter(x[i], y[i], color='red')
			plt.scatter(x_init_pos[i], y_init_pos[i], color='red')

		# plot polygon
		plt.plot(polyx, polyy, '--', color='black', linewidth=1)
		# plot Obstacle polygon
		if len(self.cartObst) > 0:
			plt.plot(obstx, obsty, '--', color='black')


		if self.optuna:
			plt.title("Optimal Initial Positions")
		else:
			plt.title("Random Initial Positions")

		# plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0)
		#plt.legend(loc='upper left')

		ax.set_aspect('equal', adjustable='box')

		plt.axis('off')
		plt.savefig("filename.png", transparent=True)
		# Merge all in one figure
		plt.show()

