#*
# * Representation of a point in the North/East/Down (NED) geographical coordinate system.
# 
class NED(object):

    def __init__(self, north, east, down):

        self.north = north
        self.east = east
        self.down = down