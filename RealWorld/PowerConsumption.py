""" AirSim provides 3 vehicle modes (can be found at https://github.com/microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/MultiRotorParams.hpp):

    1. QuadX:F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
    2. Hex: with the same dimsensions of F450 frame but with 6 rotors
    3. Octo: with the same dimsensions of F450 frame but with 8 rotors

    Every mode uses the same Rotor parameters:
    https://github.com/microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/RotorParams.hpp


    SimpleFlight mode as default uses the setupFrameGenericQuad() with the following specifications:
        - dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        - motor_assembly_weight = 0.055 # weight for MT2212 motor for F450 frame (https://tmkarc1hobby.com/tiger-motor-mt2212-13-980kv-brushless-motor-tm-mt2212-13.html)
        - 4 GWS 9x5 propeller for which C_T = 0.109919, C_P = 0.040164 @ 6396.667 RPM (propeller performance database http://m-selig.ae.illinois.edu/props/propDB.html)

        Additional info about the propellers can be found at ..\UIUC-propDB\UIUC-propDB\volume-2\data\gwsdd_9x5_static_0797rd

        RPM        CT        CP
     1430.000  0.092491  0.040151
     1756.667  0.093638  0.039669
     2033.333  0.095029  0.039341
     2240.000  0.097139  0.039812
     2490.000  0.098022  0.039858
     2746.667  0.097710  0.039239
     2986.667  0.100460  0.039886
     3296.667  0.102358  0.040045
     3493.333  0.102279  0.039731
     3736.667  0.103308  0.039703
     3980.000  0.104592  0.039897
     4196.667  0.105721  0.040037
     4460.000  0.106368  0.039906
     4713.333  0.105266  0.039318
     4956.667  0.106268  0.039585
     5183.333  0.108129  0.040049
     5436.667  0.107653  0.039728
     5706.667  0.107403  0.039539
     5936.667  0.107599  0.039539
     6186.667  0.108081  0.039601
     6396.667  0.109919  0.040164 ***

"""

""" https://en.wikipedia.org/wiki/Motor_constants """

import math

max_rpm = 6396.667  # Kv: revolutions per minute (https://github.com/microsoft/AirSim/blob/master/AirLib/include/vehicles/multirotor/RotorParams.hpp)
# Kv = (max_rpm * 2 * math.pi) / 60  # in (m/s)/ V:
Kv = 980  # rpm/V : supplied with 11.1 V will run at a nominal speed 10878 rpm
Kv = (Kv * 2 * math.pi) / 60  # in (m/s)/ V: translated to "linear"
Kt = 1 / Kv  # (N⋅m of torque per ampere of current)


class power_consumption_model:
    def __init__(self):
        self.P = 0  # power consumed

    def estimated_power(self, rotors):
        self.P = self.P + 2 * Kv * rotors.rotors_.rotors[0].torque_scaler * rotors.rotors_.rotors[0].speed + \
                 2 * Kv * rotors.rotors_.rotors[1].torque_scaler * rotors.rotors_.rotors[1].speed + \
                 2 * Kv * rotors.rotors_.rotors[2].torque_scaler * rotors.rotors_.rotors[2].speed + \
                 2 * Kv * rotors.rotors_.rotors[3].torque_scaler * rotors.rotors_.rotors[3].speed

    def final_estimated_power(self):
        return self.P
