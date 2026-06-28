#!/usr/bin/env python3
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class TestDefaultRxHookBase(common.SafetyTest):
  FWD_BUS_LOOKUP = {}

  def test_rx_hook(self):
    # default rx hook allows all msgs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)), f"failed RX {addr=}")


class TestNoOutput(TestDefaultRxHookBase):
  TX_MSGS = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.safety.init_tests()

  def test_extra_branch_coverage(self):
    self.safety.safety_tick_null()
    self.assertEqual(0, self.safety.to_signed(1, 0))
    self.assertEqual(0, self.safety.to_signed(1, -5))
    for brake, brake_prev, regen, regen_prev, steer, steer_prev, moving, expected_allowed in [
      (False, False, False, False, True, False, False, False),
      (False, False, False, False, True, True, False, True),
      (True, False, False, False, False, False, False, False),
      (True, True, False, False, False, False, False, True),
      (True, True, False, False, False, False, True, False),
      (False, False, True, False, False, False, False, False),
      (False, False, True, True, False, False, False, True),
      (False, False, True, True, False, False, True, False),
    ]:
      self.safety.set_controls_allowed(True)
      self.safety.trigger_generic_rx_checks(brake, brake_prev, regen, regen_prev, steer, steer_prev, moving)
      self.assertEqual(self.safety.get_controls_allowed(), expected_allowed, (brake, brake_prev, regen, regen_prev, steer, steer_prev, moving))

    self.assertEqual(-1, self.safety.set_safety_hooks(9999, 0))

    # Test get_checksum and compute_checksum branch paths
    for has_get, has_compute, ignore_checksum, expected_rx in [
      (True, True, False, True),
      (True, True, True, True),
      (False, True, False, False),
      (False, True, True, True),
    ]:
      self.safety.set_mock_safety_hooks(has_get, has_compute, False, False)
      self.safety.set_mock_rx_check(0x123, 0, 8, ignore_checksum, True, 0, True, 100)
      self.assertEqual(self._rx(common.make_msg(0, 0x123, 8)), expected_rx)

    # Test counter check branch paths
    for has_counter, ignore_counter, max_counter, expected_rx in [
      (True, False, 15, True),
      (True, True, 15, True),
      (False, False, 15, False),
      (False, True, 15, True),
      (True, False, 0, False),
      (True, True, 0, True),
    ]:
      self.safety.set_mock_safety_hooks(True, True, has_counter, False)
      self.safety.set_mock_rx_check(0x123, 0, 8, True, ignore_counter, max_counter, True, 100)
      self.assertEqual(self._rx(common.make_msg(0, 0x123, 8)), expected_rx)

    # Test quality flag check branch paths
    for has_qf, ignore_qf, qf_val, expected_rx in [
      (True, False, True, True),
      (True, False, False, False),
      (True, True, False, True),
      (False, False, False, False),
      (False, True, False, True),
    ]:
      self.safety.set_mock_safety_hooks(True, True, False, has_qf)
      self.safety.set_mock_rx_check(0x123, 0, 8, True, True, 0, ignore_qf, 100)
      msg = common.make_msg(0, 0x123, 8, b'\x00\x00\x01\x00\x00\x00\x00\x00' if qf_val else b'\x00\x00\x00\x00\x00\x00\x00\x00')
      self.assertEqual(self._rx(msg), expected_rx)

    # Test low frequency in safety_tick
    self.safety.set_mock_safety_hooks(True, True, False, False)
    self.safety.set_mock_rx_check(0x123, 0, 8, True, True, 0, True, 5)
    self.safety.set_timer(0)
    self.assertTrue(self._rx(common.make_msg(0, 0x123, 8)))
    self.safety.set_controls_allowed(True)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_checks_alternative_mismatch(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.nissan, 0)
    self.safety.init_tests()
    self.assertTrue(self._rx(common.make_msg(0, 0x15c, 8)))
    # This message is not matched/whitelisted because index 0 was already locked, but safety_rx_hook still returns True
    self.assertTrue(self._rx(common.make_msg(1, 0x15c, 8)))

  def test_forwarding_static_blocking(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0)
    self.safety.init_tests()
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x488))
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.safety.init_tests()
    self.safety.set_disable_forwarding(False)
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x123))



class TestSilent(TestNoOutput):
  """SILENT uses same hooks as NOOUTPUT"""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.silent, 0)
    self.safety.init_tests()


class TestAllOutput(TestDefaultRxHookBase):
  # Allow all messages
  TX_MSGS = [[addr, bus] for addr in common.SafetyTest.SCANNED_ADDRS
             for bus in range(4)]

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.allOutput, 0)
    self.safety.init_tests()

  def test_spam_can_buses(self):
    # asserts tx allowed for all scanned addrs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        should_tx = [addr, bus] in self.TX_MSGS
        self.assertEqual(should_tx, self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  def test_default_controls_not_allowed(self):
    # controls always allowed
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_hook_on_wrong_safety_mode(self):
    # No point, since we allow all messages
    pass


class TestAllOutputPassthrough(TestAllOutput):
  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.allOutput, 1)
    self.safety.init_tests()


if __name__ == "__main__":
  unittest.main()
