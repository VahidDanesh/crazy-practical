## Obstacle Avoidance and Landing Detection for Crazyflie (Pad Same Height as Launch Pad)

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = "radio://0/88/2M/E7E7E7E7F0"

logging.basicConfig(level=logging.ERROR)

# Parameters
Hovering_HEIGHT = 0.5        # meters
Forward_DISTANCE_LIMIT = 1.5 # meters forward before search
Search_AREA_SIZE = 0.6       # meters (~2ft)

# --- helper functions ---
def is_close(distance, threshold=0.4):
    if distance is None:
        return False
    return distance < threshold

def detect_landing_zone(mr, baseline, tolerance=0.1):
    """
    Detect landing pad if the downward distance matches the launch pad baseline
    within a tolerance (instead of detecting a drop).
    """
    if mr.down is None:
        return False
    return abs(mr.down - baseline) < tolerance

def boundary_detected(mr, threshold=0.4):
    """Check if any wall is too close (front/back/left/right)."""
    return (
        (mr.front is not None and mr.front < threshold) or
        (mr.back is not None and mr.back < threshold) or
        (mr.left is not None and mr.left < threshold) or
        (mr.right is not None and mr.right < threshold)
    )

def handle_boundary(mc):
    """Stop, back off, and turn around when a wall is detected."""
    print("Wall detected! Turning around.")
    mc.stop()
    time.sleep(0.5)

    # Back up slightly
    mc.start_back(0.2)
    time.sleep(1.0)
    mc.stop()
    time.sleep(0.5)

    # Turn around 180 degrees
    mc.turn_left(180)
    time.sleep(2.0)
    mc.stop()
    time.sleep(1.0)

def search_for_landing(mc, mr, baseline):
    """Search a square area for a landing zone."""
    for _ in range(2):  # two passes around the square
        for dx, dy, duration in [
            (0.0, 0.2, Search_AREA_SIZE / 0.2),   # move right
            (0.2, 0.0, Search_AREA_SIZE / 0.2),   # move forward
            (0.0, -0.2, Search_AREA_SIZE / 0.2),  # move left
            (-0.2, 0.0, Search_AREA_SIZE / 0.2),  # move back
        ]:
            mc.start_linear_motion(dx, dy, 0)
            start = time.time()
            while time.time() - start < duration:
                if detect_landing_zone(mr, baseline):
                    print("Landing pad detected! Landing...")
                    mc.stop()
                    time.sleep(1.0)
                    mc.land()
                    return True

                if boundary_detected(mr):
                    handle_boundary(mc)
                    # Retry search after turning
                    return search_for_landing(mc, mr, baseline)

                time.sleep(0.1)

    return False


# --- main flight ---
if __name__ == '__main__':
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:

        # Arm and takeoff
        scf.cf.platform.send_arming_request(True)
        time.sleep(1)

        with MotionCommander(scf, default_height=Hovering_HEIGHT) as mc:
            with Multiranger(scf) as mr:

                # Hover to measure ground (baseline pad height)
                print("Hovering 3 seconds to measure baseline pad height...")
                time.sleep(3.0)
                baseline = mr.down if mr.down is not None else 0.5
                print(f"Baseline pad height: {baseline:.2f} m")

                keep_flying = True
                landed = False
                distance_traveled = 0.0
                VELOCITY = 0.3
                last_time = time.time()

                # --- Exploration loop until 1.5m forward ---
                while keep_flying and not landed and distance_traveled < Forward_DISTANCE_LIMIT:
                    now = time.time()
                    dt = now - last_time
                    last_time = now

                    velocity_x, velocity_y = 0.0, 0.0

                    if is_close(mr.front):
                        print("Obstacle ahead → sidestepping")
                        if not is_close(mr.left):
                            velocity_y = -VELOCITY
                        elif not is_close(mr.right):
                            velocity_y = VELOCITY
                        else:
                            velocity_x = -VELOCITY  # backup
                    else:
                        velocity_x = VELOCITY

                    mc.start_linear_motion(velocity_x, velocity_y, 0)
                    time.sleep(0.1)

                    # Update distance traveled
                    distance_traveled += velocity_x * dt

                    # --- Wall detection ---
                    if boundary_detected(mr):
                        handle_boundary(mc)
                        landed = search_for_landing(mc, mr, baseline)
                        keep_flying = False
                        break

                if not landed:
                    # --- Stop forward motion ---
                    mc.stop()
                    time.sleep(1.0)
                    print("Reached 1.5m forward → starting landing pad search")

                    # --- Search routine ---
                    landed = search_for_landing(mc, mr, baseline)

                    if not landed:
                        print("No landing pad detected in search area. Hovering...")
                        mc.stop()
                        time.sleep(3.0)
                        mc.land()
                        print("Landing anyway after search.")

            print("Crazyflie has landed.")


