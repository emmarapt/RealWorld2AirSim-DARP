#*
# * Representation of a point in the Earth-Centered, Earth-Fixed (ECEF) geographic coordinate
# * system.
# * <p>
# * ECEF uses three-dimensional XYZ coordinates (in meters) to describe a location. The term
# * "Earth-Centered" comes from the fact that the origin of the axis (0,0,0) is located at the mass
# * center of gravity. The term "Earth-Fixed" implies that the axes are fixed with respect to the
# * earth. The Z-axis pierces the North Pole, and the XY-axis defines the equatorial plane.
# 
class ECEF(object):


    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

