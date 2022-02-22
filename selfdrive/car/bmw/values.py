from selfdrive.car import dbc_dict

# Steer torque limits
class SteerLimitParams: #controls running @ 100hz
  STEER_MAX = 9
  STEER_DELTA_UP = 10 / 100       # 10Nm/s
  STEER_DELTA_DOWN = 1000 / 100     # 10Nm/sample - no limit
  STEER_ERROR_MAX = 999     # max delta between torque cmd and torque motor

class SteerActuatorParams: # stepper parameters
  MAX_STEERING_TQ = 8  # Nm
  #Todo remvoe this if not used - since it is included in DBC factor
  # ACTUATOR_RATIO = 25 / 12 * (13+212/289)  # big cog / small cog * planetary gearbox
  # POSITION_SCALING = 256 / 1.8  # microsteps / base step angle [deg]
  CENTERING_COEFF = 0.0005
  ZERO_ANGLE_HOLD_TQ = 1 #carcontroller will interpolate between zero angle torque and where linear region start
  STEER_LINEAR_REGION = 5 #deg start of linear region for small angles
  STEER_TORQUE_OFFSET = 3  # add offset to due
  STEER_BACKLASH = 1 #deg


#car chimes: enumeration from dbc file. Chimes are for alerts and warnings
class CM:
  MUTE = 0
  SINGLE = 3
  DOUBLE = 4
  REPEATED = 1
  CONTINUOUS = 2


#car beepss: enumeration from dbc file. Beeps are for activ and deactiv
class BP:
  MUTE = 0
  SINGLE = 3
  TRIPLE = 2
  REPEATED = 1

class AH:
  #[alert_idx, value]
  # See dbc files for info on values"
  NONE           = [0, 0]
  FCW            = [1, 1]
  STEER          = [2, 1]
  BRAKE_PRESSED  = [3, 10]
  GEAR_NOT_D     = [4, 6]
  SEATBELT       = [5, 5]
  SPEED_TOO_HIGH = [6, 8]


class CAR:
    E82_DCC = "BMW E82 with Dynamic Cruise Control VO544 coded in"
    E90_DCC = "BMW E90 with Dynamic Cruise Control VO544"
    E82 = "BMW E82 with normal Cruise Control VO540"
    E90 = "BMW E82 with normal Cruise Control VO540"

BMW_Eseries = {
  128: 5, 201: 8, 205: 8, 206: 8, 209: 8, 212: 8, 304: 5, 320: 2, 404: 4, 470: 2, 678: 2, 884: 5, #F-CAN
  128: 5, 168: 8, 169: 8, 170: 8, 172: 8, 180: 8, 182: 5, 184: 8, 186: 8, 191: 5, 196: 7, 200: 6, 201: 8, 205: 8, 206: 8, 209: 8, 212: 8, 266: 6, 304: 5, 309: 2, 373: 3, 404: 4, 408: 5, 414: 8, 416: 8, 418: 8, 422: 8, 436: 8, 437: 7, 438: 7, 464: 8, 466: 6, 470: 2, 481: 6, 502: 2, 514: 2, 538: 3, 550: 5, 570: 4, 578: 5, 594: 2, 678: 2, 690: 8, 691: 5, 704: 3, 719: 2, 722: 3, 753: 3, 758: 2, 760: 8, 762: 5, 764: 7, 784: 7, 785: 2, 797: 2, 816: 8, 818: 2, 821: 8, 823: 2, 847: 2, 884: 5, 893: 2, 896: 7, 897: 2, 899: 4,   904: 7, 940: 2, 945: 6, 947: 6, 948: 8, 953: 3, 958: 2, 1007: 3, 1152: 8, 1170: 8, 1175: 8, 1176: 8, 1193: 8, 1246: 8, 1408: 8, 1426: 8, 1432: 8, 1449: 8, 1472: 8, 1494: 8, 1504: 8, 1506: 8, 1517: 8, 1522: 8, 1528: 8 #PT-CAN
}


FINGERPRINTS = {
  CAR.E82_DCC:  [{**BMW_Eseries, **{403: 8}}],    #add DynamicCruiseControlStatus message for VO544 option
  CAR.E82:      [{**BMW_Eseries, **{512: 8}}],    #add CruiseControlStatus message for VO540 option
}

DBC = {
  CAR.E82_DCC: dbc_dict('bmw_e9x_e8x', 'toyota_adas'),  #'toyota_adas' for potentially retrofitted radar
  CAR.E90_DCC: dbc_dict('bmw_e9x_e8x', 'toyota_adas'),
  CAR.E82: dbc_dict('bmw_e9x_e8x', 'toyota_adas'),
  CAR.E90: dbc_dict('bmw_e9x_e8x', 'toyota_adas')
}

STEER_THRESHOLD = 100  # retrofited actuator



NO_DSU_CAR = [CAR.E82_DCC, CAR.E90_DCC, CAR.E82, CAR.E90] #Indicate which cars don't have radar installed
TSS2_CAR = []