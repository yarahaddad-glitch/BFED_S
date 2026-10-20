import sys
from pathlib import Path
import numpy as np
import swift

here = Path(__file__).resolve().parent
sys.path.append(str(here))
sys.path.append(str(here.parent))

from EnviroParts_S.Full_sim import create_environment
from Light_curtain import SafetyLightCurtain
from Prison_break import PrisonBreak, ZONES
from Prisoner_movement import walk_all_prisoners, walk_one_prisoner


def put_at(env, mesh, xyz):
    """Move a prisoner to an exact (x, y, z) position."""
    T = np.array(mesh.T, float)
    T[:3, 3] = xyz
    mesh.T = T
    env.step(0.05)            # the check runs here


# ---------------- set up ----------------
env = swift.Swift()
env.launch(realtime=True)
scene = create_environment(env)
PRISONER1_START = np.array(scene["prisoner1"].T, float)   # where she was loaded

curtain = SafetyLightCurtain(env)
curtain.watch_prisoners([scene["prisoner1"], scene["prisoner2"], scene["prisoner3"]])
curtain.arm()

prison = PrisonBreak(env, scene)
prison.arm()

env.step(0.05)

# ---------------- TESTS ----------------
# True = run that test, False = skip it
TEST_CURTAIN = True
TEST_KITCHEN = True
TEST_ISOLATION = True

# Where prisoner1 appears for each zone test: (x, y, z)
KITCHEN_SPOT = (13.0, 2.0, 0.0)
ISOLATION_SPOT = (6.0, 15.0, 0.0)

if TEST_CURTAIN:
    print("\n===== TEST 1: prisoner1 walks 4 m into the light curtain =====")
    mesh = scene["prisoner1"]
    if not np.allclose(np.array(mesh.T, float), PRISONER1_START):
        print("prisoner1 isn't at her starting position - moving her back")
        mesh.T = PRISONER1_START.copy()
        env.step(0.05)
    walk_one_prisoner(env, scene, distance=4.0)

if TEST_KITCHEN:
    print(f"\n===== TEST 2: prisoner1 appears at {KITCHEN_SPOT} (kitchen) =====")
    put_at(env, scene["prisoner1"], KITCHEN_SPOT)

if TEST_ISOLATION:
    print(f"\n===== TEST 3: prisoner1 appears at {ISOLATION_SPOT} (isolation) =====")
    put_at(env, scene["prisoner1"], ISOLATION_SPOT)

print("\nAll safety tests finished.")
env.hold()