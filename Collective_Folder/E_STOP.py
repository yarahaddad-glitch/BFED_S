_handlers = []
active = False          # True while an emergency stop is being handled
_stop_callback = None


def set_stop_callback(callback):
    """Shared GUI integration; the runtime owns reset and separate resume."""
    global _stop_callback
    _stop_callback = callback


def register(home, resume):
    _handlers.append((home, resume))


def stop(message):
    global active
    if _stop_callback is not None:
        _stop_callback(message)
        return
    if active:                      # already stopping - ignore further alarms
        return
    active = True
    try:
        print(f"\n!!! {message}")
        print("!!! EMERGENCY STOP - robots going to their home position")
        saved = [home() for home, _ in _handlers]
        input(">>> Press Enter to reset and restart... ")
        for (_, resume), s in zip(_handlers, saved):
            resume(s)
        print("Restarted.")
    finally:
        active = False
