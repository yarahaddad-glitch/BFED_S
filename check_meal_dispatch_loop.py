"""Check camera/dispatch sequencing without needing Swift or a webcam."""
import ast
from itertools import permutations
from pathlib import Path
from types import SimpleNamespace
import unittest


# Load the production loop without importing the graphical simulation stack.
source = Path(__file__).parent / "Collective_Folder/EnviroParts_S/main_meal_bot.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
loop = next(node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "run_orders")
namespace = {}
exec(compile(ast.Module(body=[loop], type_ignores=[]), str(source), "exec"), namespace)
run_orders = namespace["run_orders"]


class DispatchLoopCheck(unittest.TestCase):
    def run_sequence(self, colours, *, empty=False, fail=False):
        events = []
        cards = iter([*colours, None])

        def scan():
            card = next(cards)
            events.append(("scan", card))
            return card

        def handle(card):
            action = ("CANCELLED" if card is None else
                      "ALERT" if card == "YELLOW" else "SEND_TRAY")
            return SimpleNamespace(action=action, message="",
                                   order=SimpleNamespace(tray=SimpleNamespace(colour=card)))

        def serve(colour, on_conveyor):
            events.append(("pickup", colour))
            if fail:
                raise RuntimeError("blocked route")
            if empty:
                return False
            on_conveyor()
            events.append(("handout", colour))
            return True

        dispatcher = SimpleNamespace(
            handle=handle,
            mark_on_conveyor=lambda order: events.append(("belt", order.tray.colour)),
            mark_delivered=lambda order: events.append(("delivered", order.tray.colour)),
            reset=lambda colour: events.append(("reset", colour)))
        run_orders(SimpleNamespace(scan=scan), dispatcher, SimpleNamespace(serve=serve))
        return events

    def test_any_arrival_order_and_repeated_colours(self):
        for colours in set(permutations(["GREEN", "BLUE", "GREEN", "BLUE"])):
            expected = []
            for colour in colours:
                expected.extend((stage, colour) for stage in
                                ("scan", "pickup", "belt", "handout", "delivered", "reset"))
            expected.append(("scan", None))
            self.assertEqual(self.run_sequence(colours), expected)

    def test_yellow_does_not_move(self):
        self.assertEqual(self.run_sequence(["YELLOW"]),
                         [("scan", "YELLOW"), ("scan", None)])

    def test_empty_stack_reopens_camera(self):
        self.assertEqual(self.run_sequence(["GREEN"], empty=True),
                         [("scan", "GREEN"), ("pickup", "GREEN"),
                          ("reset", "GREEN"), ("scan", None)])

    def test_motion_failure_stops_without_delivering(self):
        self.assertEqual(self.run_sequence(["BLUE", "GREEN"], fail=True),
                         [("scan", "BLUE"), ("pickup", "BLUE")])


if __name__ == "__main__":
    unittest.main()
