import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = "radio://0/88/2M/E7E7E7E7F0"

logging.basicConfig(level=logging.ERROR)


def is_close(distance, threshold=0.4):
    """Check if an obstacle is closer than threshold (in meters)."""
    if distance is None:
        return False
    return distance < threshold


def detect_landing_zone(mr, baseline, drop_threshold=0.5):
    """
    Detect a landing zone by checking if the downward sensor suddenly sees a
    lower distance (like an opening).
    baseline = normal floor distance when flying over ground
    drop_threshold = minimum extra depth to trigger landing
    """
    if mr.down is None:
        return False
    return (mr.down - baseline) > drop_threshold


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        with MotionCommander(scf, default_height=0.5) as mc:
            with Multiranger(scf) as mr:

                # --- Hover and calibrate baseline floor height ---
                print("Hovering 3 seconds to measure ground distance...")
                time.sleep(3.0)
                baseline = mr.down if mr.down is not None else 0.5
                print(f"Baseline floor distance: {baseline:.2f} m")

                keep_flying = True
                landed = False

                while keep_flying and not landed:
                    VELOCITY = 0.3
                    velocity_x = 0.0
                    velocity_y = 0.0

                    # --- Obstacle avoidance / forward motion ---
                    if is_close(mr.front):
                        print("Obstacle ahead → going around")
                        velocity_x = 0.0
                        if not is_close(mr.left):
                            velocity_y = -VELOCITY
                        elif not is_close(mr.right):
                            velocity_y = VELOCITY
                        else:
                            velocity_x = -VELOCITY  # backup
                    else:
                        velocity_x = VELOCITY  # go forward if clear

                    mc.start_linear_motion(velocity_x, velocity_y, 0)
                    time.sleep(0.1)


                    # --- Landing detection ---
                    if detect_landing_zone(mr, baseline):
                        print("Landing zone detected! Aligning...")
                        # Small pause/hover to stabilize
                        mc.stop()
                        time.sleep(1.0)

                        # Optional: center over opening with a small scan
                        scan_time = 1.5
                        mc.start_linear_motion(0.0, 0.2, 0.0)  # scan right
                        time.sleep(scan_time)
                        mc.start_linear_motion(0.0, -0.2, 0.0)  # scan left
                        time.sleep(scan_time)
                        mc.stop()

                        # Land
                        print("Descending to land...")
                        mc.land()
                        landed = True
                        break

            print("Crazyflie has landed!")

