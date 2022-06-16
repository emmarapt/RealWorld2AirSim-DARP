import math





def geo(p1, p2):
    distance = 0
    R = 6371 # Radius of the earth

    latDistance = math.radians(p2[0] - p1[0])
    lonDistance = math.radians(p2[1] - p1[1])
    a = math.sin(latDistance / 2) * math.sin(latDistance / 2) + math.cos(math.radians(p1[0])) * math.cos(math.radians(p2[0])) * math.sin(lonDistance / 2) * math.sin(lonDistance / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c * 1000 # convert to meters

    return distance


def euclidean(x1, x2):

    distance = math.sqrt((x2[1] - x1[1])*(x2[1] - x1[1]) + (x2[0] - x1[0])*(x2[0] - x1[0]))

    return distance


def getLength(WGS84, printLength):

    dist = 0
    i = 0
    while i<len(WGS84) - 1:
        dist += geo(WGS84[i], WGS84[i+1])
        i += 1
    if printLength:
        print("Path length: "+dist+" meters\n")

    return dist


def getEstimatedTime(WGS84, speed):
    dist = 0
    i = 0
    while i<len(WGS84) - 1:
        dist += geo(WGS84[i], WGS84[i+1])
        i += 1
    turnDelay =5*speed/(20+abs(speed))
    return (dist / speed + len(WGS84)*turnDelay) / 60.0
