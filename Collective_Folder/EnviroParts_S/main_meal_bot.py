"""Colour scan -> Maryam -> Yara food -> ReBot towel -> Maryam handout."""
import sys
from pathlib import Path

ASSIGNMENT_ROOT = Path(__file__).resolve().parents[2]
if str(ASSIGNMENT_ROOT) not in sys.path:
    sys.path.insert(0, str(ASSIGNMENT_ROOT))

from Maryam_Work.maryam_meal_dispatch import ColourScanner, build_default_dispatcher
from Maryam_Work.maryam_movements import build_scene
from Collective_Folder.integrated_meal import IntegratedMeal


def run_orders(scanner, dispatcher, movements):
    """Finish each meal before scanning again; stop on a failed movement."""
    while True:
        result = dispatcher.handle(scanner.scan())
        if result.action == "CANCELLED":
            print("Camera cancelled - finished.", flush=True)
            break
        if result.action == "ALERT":
            print("Yellow detected: no tray sent. Show GREEN or BLUE, or press Q to finish.",
                  flush=True)
            continue
        if result.action != "SEND_TRAY":
            print(f"[{result.action}] {result.message}", flush=True)
            continue

        order = result.order
        colour = order.tray.colour
        print(f"[MEAL] Starting {colour}: Maryam pickup, Yara food, ReBot towel, handout.", flush=True)
        try:
            served = movements.serve(
                colour, lambda: dispatcher.mark_on_conveyor(order))
        except RuntimeError as exc:
            print(f"[STOPPED] {exc}. Camera stays closed; meal is not marked delivered.",
                  flush=True)
            break
        if served:
            dispatcher.mark_delivered(order)
            print("[MEAL] Handout complete. Ready for the next colour card.", flush=True)
        dispatcher.reset(colour)


def main():
    env, objects, robot, trays = build_scene()
    movements = IntegratedMeal(env, objects, robot, trays)
    scanner = ColourScanner()
    try:
        run_orders(scanner, build_default_dispatcher(), movements)
    finally:
        scanner.close()
    env.hold()


if __name__ == "__main__":
    main()
