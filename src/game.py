import numpy as np
import pygame as pg


class DisplaySettings:
    display_size = 800, 800
    bg_color = "#C336C9"
  

class ControlState:
    is_running = False
    is_paused = False
    framerate = 20
    
class SimState:
    particle_positions = None
    particle_velocities = None


def main():
    pg.init()
    screen = pg.display.set_mode(DisplaySettings.display_size)
    clock = pg.time.Clock()
    ControlState.is_running = True


    while ControlState.is_running:
        clock.tick(ControlState.framerate)
        screen.fill(DisplaySettings.bg_color)

        for event in pg.event.get():
            if event.type == pg.QUIT:
                ControlState.is_running = False
                return
    
        pg.display.update()

if __name__ == "__main__":
    main()
