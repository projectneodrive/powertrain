'''===================================================================================
=   Title	: Odrive configuration for BLDC with hall sensor with power supply.	
--------------------------------------------------------------------------------------
=   Auteur:         Timothée Roth					
=   Version:        V 1.0								               		                            			
=   Date :          02.04.26	                                      
=   Modification: 	Initial commit		
=----------------------------------------------------------------------------------
=   General description:						
= ------------------------------------
 *  1) In the configuration section, adapt settings to your motor & power supply. 
 *  2) Launch with python3 Odrive_configuration.py
 *  3) Do step by step configuration (0->5) or automatic (9). Be carefue to 
 *      have the motor free on its axis — it will spin and make noise!
 *  4) Once the configuration done with no error, test the motor with (5). 
 *      Use a small value of torque (3-8 Nm) for the test. The motor will stop 
 *      automatically.
==================================================================================='''

#command in local folder for acces to python drive api
'''
python3 -m venv venv
source venv/bin/activate
pip install odrive
python3 Odrive_configuration.py
'''

import odrive
from odrive.enums import (
    AxisState,
    ControlMode,
    InputMode,
    EncoderMode,
    MotorType,
)
import time
import threading

# ── Configuration ─────────────────────────────────────────────────────────────
# Motor
POLE_PAIRS              = 25        # Motor pole pairs
KV_RATING               = 9.38      # Motor KV (used to compute torque constant)
TORQUE_LIM              = 50        # Motor torque limit (Nm)
RESISTANCE_CALIB_MAX_V  = 30        # Max voltage during resistance calibration (V)
CALIBRATION_CURRENT     = 8         # Calibration current (A)
CURRENT_LIM             = 15        # Motor current limit (A)
REQUESTED_CURRENT_RANGE = 29       #Only above 60A # Requested current range (A)
# Encoder
ENCODER_CPR             = 6*POLE_PAIRS # Encoder counts per revolution, 
CALIB_RANGE             = 0.3       # Calibration range for encoder
# Controller
POS_GAIN                = 0#20        # Position gain
VEL_GAIN                = 0.5      # Velocity gain
VEL_INTEGRATOR_GAIN     = 0#.1       # Velocity integrator gain
VEL_LIMIT               = 100        # Velocity limit (turns/s) LET BIG VAL
TORQUE_RAMP_RATE        = 30        # Torque ramp rate (Nm/s)
# General
BRAKE_RESISTANCE        = 2         # Brake resistor value (Ohm)
DC_MAX_POSITIVE_CURRENT = 15        # Max DC positive current (A)
#DC_MAX_NEGATIVE_CURRENT = 0.01      # Max DC negative current (A)
# Battery 
MAX_REGEN_CURRENT       = 0      # Max regenerative current (A)
'''DC_BUS_OVERVOLTAGE      = 54.6      # Maximum DC bus voltage before fault (a)
DC_BUS_UNDERVOLTAGE     = 42        # Minimum DC bus voltage before fault (A)
DC_BUS_OVERVOLTAGE_RAMP_START = 0   # Not use now
DC_BUS_OVERVOLTAGE_RAMP_END = 0     # Not use now'''
# ──────────────────────────────────────────────────────────────────────────────

error_flag = False
odrv = None             # Global ODrive reference so all threads share it


# ── Connection ────────────────────────────────────────────────────────────────

def connect():
    global odrv
    print("Searching for ODrive...")
    odrv = odrive.find_any()
    print(f"Connected to ODrive (serial: {odrv.serial_number})")
    return odrv


def wait_and_reconnect(reason="Rebooting ODrive"):
    """Wait for ODrive to reboot and reconnect. Handles the disconnect exception."""
    print(f"{reason} — waiting for reconnection...", flush=True)
    time.sleep(4)  # Give the device time to reboot
    for attempt in range(10):
        try:
            connect()
            return True
        except Exception:
            print(f"  Reconnect attempt {attempt + 1}/10...", flush=True)
            time.sleep(2)
    print("✘ Could not reconnect to ODrive after reboot.")
    return False

def reboot():
    print("Rebooting ODrive...")
    try:
        odrv.reboot()
    except Exception:
        pass  # Disconnect during reboot is expected
    wait_and_reconnect("Reboot complete")
    print("✔ ODrive rebooted and reconnected.")

# ── Configuration steps ───────────────────────────────────────────────────────

def erase_config():
    print("Erasing ODrive configuration and rebooting...")
    try:
        odrv.erase_configuration()
    except Exception:
        pass  # Disconnect during erase is expected — ODrive reboots immediately
    wait_and_reconnect("Erase complete")
    print("✔ Reconnected after erase.\n")

def general_config():
    #odrv.config.enable_brake_resistance = True
    odrv.config.brake_resistance = BRAKE_RESISTANCE
    odrv.config.dc_max_positive_current = DC_MAX_POSITIVE_CURRENT
    #odrv.config.dc_max_negative_current = DC_MAX_NEGATIVE_CURRENT
    odrv.config.max_regen_current = MAX_REGEN_CURRENT
    #odrv.config.dc_bus_overvoltage_trip_level = DC_BUS_OVERVOLTAGE #NEW
    #odrv.config.dc_bus_undervoltage_trip_level = DC_BUS_UNDERVOLTAGE #NEW
    #odrv.config.dc_bus_overvoltage_ramp_start = DC_BUS_OVERVOLTAGE_RAMP_START
    #odrv.config.dc_bus_overvoltage_ramp_end = DC_BUS_OVERVOLTAGE_RAMP_END
    odrv.save_configuration()
    print("✔ General configuration complete.")


def motor_config():
    odrv.axis0.motor.config.pole_pairs = POLE_PAIRS
    odrv.axis0.motor.config.torque_constant = 8.27 / KV_RATING
    odrv.axis0.motor.config.torque_lim = TORQUE_LIM
    odrv.axis0.motor.config.motor_type = MotorType.HIGH_CURRENT
    odrv.axis0.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_V
    odrv.axis0.motor.config.calibration_current = CALIBRATION_CURRENT
    odrv.axis0.motor.config.current_lim = CURRENT_LIM
    odrv.axis0.motor.config.requested_current_range = REQUESTED_CURRENT_RANGE
    odrv.save_configuration()
    print("✔ Motor configuration complete.")


def encoder_config():
    odrv.axis0.encoder.config.mode = EncoderMode.HALL
    odrv.axis0.encoder.config.cpr = ENCODER_CPR
    odrv.axis0.encoder.config.calib_range = CALIB_RANGE
    odrv.save_configuration()
    print("✔ Encoder configuration complete.")


def controller_config():
    odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    #odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH #NEW
    odrv.axis0.controller.config.pos_gain = POS_GAIN
    odrv.axis0.controller.config.vel_gain = VEL_GAIN
    odrv.axis0.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN
    odrv.axis0.controller.config.vel_limit = VEL_LIMIT
    odrv.axis0.controller.config.torque_ramp_rate = TORQUE_RAMP_RATE
    odrv.save_configuration()
    print("✔ Controller configuration complete.")


def full_config():
    """Run all configuration steps in order."""
    print("\n── Running full configuration ──")
    general_config()
    motor_config()
    encoder_config()
    controller_config()
    print("✔ Full configuration complete.\n")


# ── Calibration ───────────────────────────────────────────────────────────────

def wait_for_idle(timeout=60):
    """Poll axis state until it returns to IDLE (calibration done) or timeout."""
    print("  Waiting for calibration to complete...", end="", flush=True)
    deadline = time.time() + timeout
    while time.time() < deadline:
        if odrv.axis0.current_state == AxisState.IDLE:
            print(" done.")
            return True
        print(".", end="", flush=True)
        time.sleep(0.5)
    print(" timeout!")
    return False


def physical_calibration():
    global error_flag

    print("\n── Motor calibration ──")
    print("The motor must be free on its axis — it will spin and make noise!")
    odrv.axis0.requested_state = AxisState.MOTOR_CALIBRATION

    if not wait_for_idle(timeout=60):
        print("✘ Motor calibration timed out.")
        error_flag = True
        return

    error = odrv.axis0.motor.error
    if error != 0:
        print(f"✘ Motor calibration error: {error}")
        error_flag = True
        return

    odrv.axis0.motor.config.pre_calibrated = True
    odrv.save_configuration()
    print("✔ Motor calibration complete.")
    
    reboot()

    initial_range = CALIB_RANGE
    while not odrv.axis0.encoder.config.pre_calibrated:
        print("\n── Encoder offset calibration ──")
        odrv.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
        odrv.axis0.encoder.config.calib_range = initial_range

        if not wait_for_idle(timeout=60):
            print("✘ Encoder calibration timed out.")
            error_flag = True
            return

        error = odrv.axis0.encoder.error
        if error != 0:
            print(f"✘ Encoder calibration error: {error}")
            error_flag = True
            return

        # Save pre_calibrated BEFORE rebooting so it persists
        odrv.axis0.encoder.config.pre_calibrated = True
        odrv.save_configuration()
        initial_range += 0.05

    print("✔ Encoder calibration complete — settings saved.")
    print("── Closed loop control enable ──")
    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv.axis0.config.startup_closed_loop_control = True
    odrv.save_configuration()
    reboot()

    


# ── Axis control ─────────────────────────────────────────────────────────────

def error_checks():
    
    print("\n── Error checks ──")
    # ── Pre-checks: diagnose what could block the transition ──
    ready = True

    motor_err = odrv.axis0.motor.error
    if motor_err != 0:
        print(f"  ✘ motor.error = {motor_err} — motor has an unresolved error")
        ready = False
 
    encoder_err = odrv.axis0.encoder.error
    if encoder_err != 0:
        print(f"  ✘ encoder.error = {encoder_err} — encoder has an unresolved error")
        ready = False
 
    if not odrv.axis0.motor.is_calibrated:
        print("  ✘ Motor is not calibrated — run physical calibration first (option 2)")
        ready = False
    
    # Hall encoders do not set is_ready like incremental encoders — check pre_calibrated instead
    if not odrv.axis0.encoder.config.pre_calibrated :
        print("  ✘ Encoder is not pre-calibrated — run physical calibration first (option 2)")
        ready = False
 
    if not ready:
        print("✘ Cannot enter closed loop control — fix the above issues first.")
        return
    

    axis_error    = odrv.axis0.error
    motor_error   = odrv.axis0.motor.error
    encoder_error = odrv.axis0.encoder.error

    if axis_error != 0 or motor_error != 0 or encoder_error != 0:
        print(f"✘ Closed loop control failed:")
        print(f"   axis0.error    = {axis_error}")
        print(f"   motor.error    = {motor_error}")
        print(f"   encoder.error  = {encoder_error}")
        print(f"   motor.pre_calibrated   = {odrv.axis0.motor.config.pre_calibrated}")
        print(f"   encoder.pre_calibrated = {odrv.axis0.encoder.config.pre_calibrated}")
        print(f"   current axis state     = {odrv.axis0.current_state}")
    else:
        odrv.axis0.config.startup_closed_loop_control = True
        odrv.save_configuration()
        print("✔ No error. Motor is no fully configure.")


# ── Torque test ───────────────────────────────────────────────────────────────

def testing_torque(Trq_test):
    print(f"input torque for testing {Trq_test} Nm")
    odrv.axis0.controller.input_torque = Trq_test
    time.sleep(5)
    odrv.axis0.controller.input_torque = 0
    time.sleep(2)


# ── Input thread ──────────────────────────────────────────────────────────────

def print_menu():
    print("\n┌─────────────────────────────────────────┐")
    print("│  0 → Erase configuration                │")
    print("│  1 → Full configuration                 │")
    print("│  2 → Physical calibration               │")
    print("│  3 → Error checks                       │")
    print("│  4 → Reboot                             │")
    print("│  5 → Torque value for test              │")
    print("│  9 → 0-1-2-3-4-5@5Nm                    │")
    print("│  q → Quit                               │")
    print("└─────────────────────────────────────────┘")
    print("Choice: ", end="", flush=True)


def input_thread():
    global error_flag
    print_menu()
    while not error_flag:
        try:
            raw = input().strip().lower()

            if raw == "0":
                erase_config()
            elif raw == "1":
                full_config()
            elif raw == "2":
                physical_calibration()
            elif raw == "3":
                error_checks()
            elif raw == "4":
                reboot()
            elif raw == "5":
                torque = float(input("  Torque value (Nm): "))
                testing_torque(torque)
            elif raw == "9":
                erase_config()
                full_config()
                physical_calibration()
                error_checks()
                reboot()
                testing_torque(5)
            elif raw == "q":
                print("Exiting.")
                error_flag = True
                break
            else:
                print(f"  ⚠ Unknown command '{raw}'.")

            if not error_flag:
                print_menu()

        except EOFError:
            break
        except Exception as e:
            print(f"  ⚠ Unexpected error: {e}")
            break


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    connect()

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    # Keep main thread alive so the program doesn't exit immediately
    while not error_flag:
        time.sleep(0.2)
