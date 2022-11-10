import numpy as np
import pygame as pg


class DisplaySettings:
    display_size = 800, 800
    bg_color = "#C336C9"
  

class SimulationState:
    is_running = False
    is_paused = False


def main():
    pg.init()
    screen = pg.display.set_mode(DisplaySettings.display_size)
    clock = pg.time.Clock()
    SimulationState.is_running = True


    while SimulationState.is_running:
        screen.fill(DisplaySettings.bg_color)

        for event in pg.event.get():
            if event.type == pg.QUIT:
                is_running = False
                return
    
        pg.display.update()

if __name__ == "__main__":
    main()
