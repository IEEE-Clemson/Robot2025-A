import commands2
from time import sleep, time
from subsystems.vision import Vision, VisionConfig
vision_config = VisionConfig()
vision_config.should_display = True
vision_config.dev_index = 4
vision = Vision(vision_config)
def update_thread():
    t = time()
    dt = 0.05
    commands2.CommandScheduler.getInstance().enable()

    while True:
        target_t = 0.02 - dt
        if target_t > 0:
            sleep(target_t)
        commands2.CommandScheduler.getInstance().run()
        t_new = time()
        dt = t_new - t
        t = t_new

update_thread()
