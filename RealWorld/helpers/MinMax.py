import sys
from numba import njit

@njit
def indMaxNodeMinArea(nodesIn, area):
	if len(nodesIn) != len(area) or len(nodesIn) == 0 or len(area) == 0:
		return -1  # null/empty or unequal length of matrices

	returnIND = 0
	largest = -sys.maxsize
	bestArea = sys.maxsize
	for i in range(len(nodesIn)):
		if nodesIn[i] > largest:
			largest = nodesIn[i]
			bestArea = area[i]
			returnIND = i
		elif nodesIn[i] == largest and area[i] < bestArea:
			bestArea = area[i]
			returnIND = i

	return returnIND

@njit
def indMax(array):
	if array is None or len(array) == 0:
		return -1

	largest = 0
	for i in range(len(array)):
		if array[i] > array[largest]:
			largest = i

	return largest

@njit
def indMin(array):
	if array is None or len(array) == 0:
		return -1

	smallest = 0
	for i in range(len(array)):
		if array[i] < array[smallest]:
			smallest = i

	return smallest

@njit
def xMax(polygonCoordinates):
	l = len(polygonCoordinates)
	xMax = -sys.maxsize

	for i in range(l):
		if polygonCoordinates[i][0] > xMax:
			xMax = polygonCoordinates[i][0]

	return xMax

@njit
def yMax(polygonCoordinates):
	l = len(polygonCoordinates)
	yMax = -sys.maxsize

	for i in range(l):
		if polygonCoordinates[i][1] > yMax:
			yMax = polygonCoordinates[i][1]

	return yMax

@njit
def xMin(polygonCoordinates):
	l = len(polygonCoordinates)
	xMix = sys.maxsize

	for i in range(l):
		if polygonCoordinates[i][0] < xMix:
			xMix = polygonCoordinates[i][0]

	return xMix

@njit
def yMin(polygonCoordinates):
	l = len(polygonCoordinates)
	yMix = sys.maxsize

	for i in range(l):
		if polygonCoordinates[i][1] < yMix:
			yMix = polygonCoordinates[i][1]

	return yMix