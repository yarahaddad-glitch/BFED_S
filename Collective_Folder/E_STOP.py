_handlers = []
active = False          # True while an emergency stop is being handled
 
 
def register(home, resume):
    _handlers.append((home, resume))
 
 
def stop(message):
    global active
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
 