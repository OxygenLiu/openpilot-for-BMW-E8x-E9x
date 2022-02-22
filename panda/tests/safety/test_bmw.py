#!/usr/bin/env python3
import unittest
import numpy as np
from panda import Panda
from panda.tests.safety import libpandasafety_py
from panda.tests.safety.common import make_msg, test_manually_enable_controls_allowed, test_spam_can_buses

MS_TO_KPH = 3.6
SAMPLING_FREQ = 100 #Hz

ANGLE_MAX_BP = [5., 15., 30]  #m/s
ANGLE_MAX = [200., 20., 10.]  #deg
ANGLE_RATE_BP = [0., 5., 15.]      # m/s
ANGLE_RATE_WINDUP = [500., 80., 15.]     #deg/s windup rate limit
ANGLE_RATE_UNWIND = [500., 350., 40.]  #deg/s unwind rate limit

ENABLED_ACTUATOR = 0 # GMLAN_HIGH 12V-> thru NPN -> ENN_pin=0V -> Trinamic drive stage enabled
DISABLED_ACTUATOR = 1 # GMLAN_LOW 0V-> thru NPN -> ENN_pin=5V -> Trinamic drive stage disabled

TX_MSGS = [[0x194, 0],[0x194, 1], [0xFC, 2]]

ACTUATOR_RATIO = 25 / 12 * (26 + 103/121)
POSITION_SCALING = 256 / 1.8 

def twos_comp(val, bits):
  if val >= 0:
    return val
  else:
    return (2**bits) + val

def sign(a):
  if a > 0:
    return 1
  else:
    return -1


def _trinamic_val_msg(to_send, cmd, cmdtype, bank, val):
  checksum = (cmd + cmdtype + bank + val) % 255
  #big endian for val:
  to_send[0].RDLR = ((cmd & 0xFF) | ((cmdtype & 0xFF) << 8) | ((bank & 0xFF) << 16) | (val & 0xFF000000 ))  
  to_send[0].RDHR = ((val & 0x00FF0000)>>16) | (val & 0x0000FF00) | ((val & 0x000000FF)<<16) | ((checksum & 0xFF)<<24)
  return to_send

  
class TestBmwSafety(unittest.TestCase):
  @classmethod
  def setUp(cls):
    cls.safety = libpandasafety_py.libpandasafety
    cls.safety.set_safety_hooks(Panda.SAFETY_BMW, 0)
    cls.safety.init_tests_bmw()

  def _angle_meas_msg(self, angle, angle_rate):
    to_send = make_msg(0, 0xc4, 7)
    
    angle_int = int(angle / 0.0439453125)
    angle_t = twos_comp(angle_int, 16) # signed

    angle_rate_int = int(angle_rate / 0.0439453125)
    angle_rate_t = twos_comp(angle_rate_int, 16) # signed

    to_send[0].RDLR = (angle_t & 0xFFFF ) | ((angle_rate_t & 0x00FF) << 24)
    to_send[0].RDHR = (angle_rate_t & 0xFF00) >> 8

    return to_send

  def _set_prev_angle(self, t):
    t = int(t * -100)
    self.safety.set_bmw_desired_angle_last(t)


    
  def _actuator_angle_cmd_msg(self, angle_delta):
    #Trinamic options:
    to_send = make_msg(2, 252)
    cmd = 4 #MVP
    cmdtype = 1  #Relative
    val = twos_comp(int(angle_delta * ACTUATOR_RATIO * POSITION_SCALING), 32) # signed
    return _trinamic_val_msg(to_send, cmd, cmdtype, 0, val)


  def _actuator_curr_cmd_msg(self, steer_current):
    #Trinamic options:
    to_send = make_msg(2, 253)
    cmd = 5  #SAP
    cmdtype = 6  #_APs.MaxCurrent
    MAX_CURRENT = 1.2
    val = int(steer_current / MAX_CURRENT * 255 ) & 0xFF
    return _trinamic_val_msg(to_send, cmd, cmdtype, 0, val)

  def _speed_msg(self, speed):
    to_send = make_msg(0, 0x1a0)
    speed = int(speed / 0.103)
    to_send[0].RDLR = (speed & 0xFFF)

    return to_send

  def _brake_msg(self, brake):
    to_send = make_msg(0, 168)
    to_send[0].RDHR = (brake * 0x3) << (61-32)

    return to_send

  def _cruise_button_msg(self, buttons_bitwise): #todo: read creuisesate
    to_send = make_msg(0, 404, 4)
    const_0xFC = 0xFC
    buttons_bitwise = buttons_bitwise & 0xFF
    if (buttons_bitwise != 0): #if any button pressed
      request_0xF = 0xF
    else:
      request_0xF = 0x0

    if (buttons_bitwise & (1<<7 | 1<<4)): #if any cancel pressed 
      notCancel = 0x0
    else:
      notCancel = 0xF

    to_send[0].RDLR = (buttons_bitwise << 16) | (request_0xF << 12) | (notCancel << 4) | (const_0xFC << 24)
    return to_send

  def test_spam_can_buses(self):
    test_spam_can_buses(self, TX_MSGS)
    
  def test_default_controls_not_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())

  def test_manually_enable_controls_allowed(self):
    test_manually_enable_controls_allowed(self)


  def test_angle_cmd_when_enabled(self): #todo add faulty BMW angle sensor (step angle)
    # when controls are allowed, angle cmd rate limit is enforced
    speeds = [ 5, 10, 15, 50, 100] #kph
    for s in speeds:
      max_angle      = np.interp(int(s / 0.103)*0.103 / MS_TO_KPH, ANGLE_MAX_BP, ANGLE_MAX) #deg
      max_delta_up   = np.interp(int(s / 0.103)*0.103 / MS_TO_KPH, ANGLE_RATE_BP, ANGLE_RATE_WINDUP) #deg
      max_delta_down = np.interp(int(s / 0.103)*0.103 / MS_TO_KPH, ANGLE_RATE_BP, ANGLE_RATE_UNWIND) #deg
      # use integer rounded value for interpolation ^^, same as what panda will receive

      self.safety.set_controls_allowed(1)
      self.safety.set_gmlan_digital_output(ENABLED_ACTUATOR)
      self.assertTrue(self.safety.get_controls_allowed())
      self.safety.safety_rx_hook(self._speed_msg(s)) #receive speed which triggers angle limits to be updated to be later used by tx 
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(self.safety.get_gmlan_digital_output(), ENABLED_ACTUATOR)
      
      # Stay within limits
      # Up
      self.safety.safety_rx_hook(self._angle_meas_msg(max_angle, max_delta_up))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(self.safety.get_gmlan_digital_output(), ENABLED_ACTUATOR)

      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(min(max_angle, max_delta_up))),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_up))
      self.assertTrue(self.safety.get_controls_allowed())

      # Stay within limits
      # Down
      self.safety.safety_rx_hook(self._angle_meas_msg(-max_angle, -max_delta_down))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(self.safety.get_gmlan_digital_output(), ENABLED_ACTUATOR)

      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(-min(max_angle, max_delta_down))),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_down))
      self.assertTrue(self.safety.get_controls_allowed())

      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(0)))
      self.assertTrue(self.safety.get_controls_allowed())
      
      # Up
      # # Inject too large measured angle
      self.safety.safety_rx_hook(self._angle_meas_msg(max_angle+1, max_delta_up))
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertEqual(self.safety.get_gmlan_digital_output(), DISABLED_ACTUATOR)
      
      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(0)))
      self.assertTrue(self.safety.get_controls_allowed())

      # Up
      # Inject too high measured rate
      self.safety.safety_rx_hook(self._angle_meas_msg(max_angle, max_delta_up+1))
      self.assertFalse(self.safety.get_controls_allowed(),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_down))
      self.assertEqual(self.safety.get_gmlan_digital_output(), DISABLED_ACTUATOR)
      
      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(0)))
      self.assertTrue(self.safety.get_controls_allowed())
      
      # Up
      # Inject too high command rate
      self.assertEqual(0, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(min(max_angle, max_delta_up) + 1.)),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_up))
      self.assertFalse(self.safety.get_controls_allowed())

      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(0)))
      self.assertTrue(self.safety.get_controls_allowed())

      
      # Down
      # Inject too large measured angle
      self.safety.safety_rx_hook(self._angle_meas_msg(-max_angle-1, -max_delta_down))
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertEqual(self.safety.get_gmlan_digital_output(), DISABLED_ACTUATOR)
      
      # Reset to 0 angle
      self.safety.set_controls_allowed(1)
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(1, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(0)))
      self.assertTrue(self.safety.get_controls_allowed())

      #Down
      # Inject too high measured rate
      self.safety.safety_rx_hook(self._angle_meas_msg(-max_angle, -max_delta_down - 1))
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertEqual(self.safety.get_gmlan_digital_output(), DISABLED_ACTUATOR)

      #Down
      # Inject too high command rate
      self.assertEqual(0, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(-min(max_angle, max_delta_down)-1.)),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_down))
      self.assertFalse(self.safety.get_controls_allowed())

      # Check desired steer should be the same as steer angle when controls are off
      self.safety.set_controls_allowed(0)
      self.assertEqual(0, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(0)),\
          'Speed: %f, Angle: %f, Delta: %f' % (s, max_angle, max_delta_down))

  def test_angle_cmd_when_disabled(self):
    self.safety.set_controls_allowed(0)

    self._set_prev_angle(0)
    self.assertFalse(self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(0)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_disengage(self):
    self.safety.set_controls_allowed(1)
    self.safety.set_gmlan_digital_output(ENABLED_ACTUATOR)
    self.safety.safety_rx_hook(self._brake_msg(0))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.get_gmlan_digital_output(), ENABLED_ACTUATOR)
    
    
    self.safety.safety_rx_hook(self._speed_msg(10)) #ALLOW_DEBUG keeps the actuator active even at 0 speed
    self.safety.safety_rx_hook(self._brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.get_gmlan_digital_output(), DISABLED_ACTUATOR)

  def test_cruise_buttons(self):
    self.safety.set_controls_allowed(1)
    self.safety.set_gmlan_digital_output(ENABLED_ACTUATOR)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.get_gmlan_digital_output(), ENABLED_ACTUATOR)
    
    self.safety.safety_rx_hook(self._cruise_button_msg(0x0)) # No button pressed
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.get_gmlan_digital_output(), ENABLED_ACTUATOR)

    self.safety.safety_rx_hook(self._speed_msg(10)) #ALLOW_DEBUG keeps the actuator active even at 0 speed
    self.safety.safety_rx_hook(self._cruise_button_msg(0x10)) # Cancel button
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.get_gmlan_digital_output(), DISABLED_ACTUATOR)

    self.safety.safety_rx_hook(self._cruise_button_msg(0x0)) # No button pressed
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.get_gmlan_digital_output(), DISABLED_ACTUATOR)

if __name__ == "__main__":
  unittest.main()
