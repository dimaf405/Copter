# CyphalCAN ESC support

This driver implements the ESC message subset of `CyphalCAN协议-V1.0.2-2024`: throttle subjects 6152 and 6153, feedback subjects 6160 and 6161, and heartbeat subject 7509. It uses Cyphal/CAN v1 message framing on classic 29-bit CAN. It does not implement the document's register, node-information, file-transfer, firmware-update, or node-ID-setting services. It does not share a CAN bus with DroneCAN v0 devices.

For X7 Pro flashing, Octa frame setup, motor mapping, and propeller-free bench testing, see [CUAV X7 Pro 使用与电机测试指南](CUAV-X7-Pro-八轴使用与电机测试.md).

## CUAV X7 Pro setup

The CUAV X7 Pro board definition exposes CAN1 and CAN2. Its default parameters use CAN1 for DroneCAN power monitoring. Keep that connection if the power module needs it. CAN2 shares pins with USB high-speed mode in this board definition; check the actual wiring and USB mode before using CAN2.

For a dedicated CAN2 ESC bus at the document's default 500 kbit/s:

```text
CAN_P2_DRIVER    2
CAN_P2_BITRATE   500000
CAN_D2_PROTOCOL  15
CAN_D2_CY_NODE   10
CAN_D2_CY_ESC_ID 16
CAN_D2_CY_ESC_RT 200
CAN_D2_CY_ESC_BM 0
```

`CAN_D2_CY_ESC_ID=16` expects motor 1 through 8 to report as node IDs `0x10` through `0x17` in that order. Identify each ESC separately before enabling all eight; its existing node ID can be read from the source field of a 7509 heartbeat or 6160/6161 feedback frame, as described in the X7 Pro guide. The document's broadcast node-ID command can change every connected ESC at once; this driver intentionally does not send that command. `CAN_D2_CY_ESC_BM` defaults to zero to avoid sending throttle until the mapping is checked. For a single-ESC bench test, set the mask to `1` and run Motor Test A without propellers; do not attempt normal arming because pre-arm checks only cover selected ESCs. Once all eight IDs and physical positions have been checked, set the mask to `255`, reboot, and run Motor Test A through H. The node ID, ESC base ID, and ESC mask are fixed at startup; edits during operation require a reboot and do not alter active motor output.

The driver reads ArduPilot motor functions 1 through 8 from `SRV_Channels`. Keep those functions assigned and verify their order with Copter Motor Test. If PWM signal wires are also connected to the ESCs, the same motor functions can reach the ESCs through both interfaces; use only the intended signal connection. Set `CAN_D2_CY_POLES` to the motor's pole-pair count if mechanical RPM telemetry is needed. Zero leaves RPM telemetry disabled because the ESC reports electrical frequency.

The driver sends zero throttle while disarmed, with the safety switch engaged, on emergency stop, or when motor output updates are older than 200 ms. Its pre-arm check requires selected ESCs to return recent 6160, 6161 and 7509 messages, CAN throttle source, zero output throttle, operational heartbeat, and no reported fault or running state. Feedback loss during flight does not trigger an automatic vehicle action. Motor stopping after a CAN cable break depends on the ESC firmware's throttle-loss timeout, which this document does not specify.

## Tests

1. Run `./waf configure --board sitl`, `./waf --targets tests/test_protocol`, `./build/sitl/tests/test_protocol`, and `Tools/autotest/autotest.py --timeout 180 build.Copter test.CyphalCAN` for protocol and virtual-bus checks.
2. Run `./waf configure --board CUAV-X7 && ./waf copter` for the target firmware. Restore the SITL configuration before running further SITL tests.
3. With propellers removed, power one ESC at a time. Confirm node ID, 500 kbit/s, 6160/6161/7509 feedback, zero throttle, and pre-arm rejection when feedback or fault state is wrong.
4. With all eight ESCs connected and propellers removed, run Copter Motor Test at low throttle. Record CAN subjects 6152/6153 and confirm each motor position, direction, commanded throttle and returned status. Test safety switch, emergency stop, stale output, CAN interruption, and the ESC's own throttle-loss timeout before flight.

SITL CAN traffic does not model physical bus termination, arbitration under load, ACK loss, bus-off, or ESC firmware watchdog behavior. These require a real CAN analyzer and powered bench testing without propellers.

The current `test.CyphalCAN` configures CyphalCAN on CAN1/D1. It does not verify the X7 Pro installation described above, which keeps a DroneCAN power module on CAN1/D1 and uses CAN2/D2 for the ESCs. Verify that two-bus combination on the target board before flight.
