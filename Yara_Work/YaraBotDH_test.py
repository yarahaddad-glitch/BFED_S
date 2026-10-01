import swift

from YaraBot_DH import YaraBot


env = swift.Swift()
env.launch(realtime=True, browser=None)

robot = YaraBot()

robot.add_to_env(env)

env.step()

input("Press Enter to close")