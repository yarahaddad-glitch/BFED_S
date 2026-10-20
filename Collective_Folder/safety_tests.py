import sys
from pathlib import Path
 
import numpy as np
import swift
from spatialmath import SE3
 
here = Path(__file__).resolve().parent
sys.path.append(str(here))
sys.path.append(str(here.parent))
 
from EnviroParts_S.Full_sim import create_environment
from Light_curtain import SafetyLightCurtain
try:
    from Prison_break import PrisonBreak
except ImportError:
    from Prison_break import PrisonBreak
 
 
def spawn_prisoner(env, prison, start, end, model_from="prisoner1"):
    """Spawn a new prisoner (same model as model_from), watch it, walk it start -> end."""
    from spatialgeometry import Mesh
    template = prison.prisoners[model_from]
    mesh = Mesh(template.filename)
    mesh.scale = template.scale
    mesh.T = SE3(start[0], start[1], np.asarray(template.T, float)[2, 3]).A
    env.add(mesh)
    prison.prisoners[f"test_prisoner_{len(prison.prisoners) + 1}"] = mesh
    walk(env, mesh, start, end)
 
 
def walk(env, mesh, start, end, speed=0.6, dt=0.05):
    """Walk a mesh in a straight line from start to end (x, y)."""
    z = np.asarray(mesh.T, float)[2, 3]
    heading = np.arctan2(end[1] - start[1], end[0] - start[0])
    steps = max(int(np.hypot(end[0] - start[0], end[1] - start[1]) / speed / dt), 1)
    for s in np.linspace(0, 1, steps + 1):
        mesh.T = (SE3(start[0] + s * (end[0] - start[0]),
                      start[1] + s * (end[1] - start[1]), z) * SE3.Rz(heading)).A
        env.step(dt)
 
 
# ---------------- set up ----------------
env = swift.Swift()
env.launch(realtime=True)
scene = create_environment(env)
 
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

if TEST_CURTAIN:
    print("\n===== TEST 1: prisoner walks into the safety light curtain =====")
    walk(env, scene["prisoner1"], (12.5, 3.6), (12.5, 5.5))

if TEST_KITCHEN:
    print("\n===== TEST 2: new prisoner walks into the kitchen =====")
    spawn_prisoner(env, prison, (13.75, 4.0), (13.75, 1.9))

if TEST_ISOLATION:
    print("\n===== TEST 3: new prisoner walks into the isolation area =====")
    spawn_prisoner(env, prison, (10.5, 15.0), (4.85, 16.4))

print("\nAll safety tests finished.")
env.hold()