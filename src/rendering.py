"""Functionality related to rendering the finished simulation"""
from __future__ import annotations
from typing import TYPE_CHECKING

from dataclasses import dataclass

import pygame as pg

if TYPE_CHECKING:
    from state import State


@dataclass
class RenderState:
    display_size = 800, 800
    bg_color = "#C336C9"
    is_running = False
    is_paused = False
    framerate = 20


def get_time_step(i_frame: int, frame_rate: int, time_inc: float) -> int:
    """Get the time step corresponding to actual time passed of simulation.

    Example:
        50 fps -> 0.02s per frame
        0.01 time_inc -> 2 time steps per frame
        3 frames -> 6 time_steps

    Args:
        i_frame: The frame currently being rendered:
        frame_rate: Number of frames rendered per second.
        time_inc: The time increments of the simulation.
    """
    return (i_frame / frame_rate) // time_inc


def render_simulation(history: list[State], time_inc: float) -> None:
    """Given a finished simulation, run over each frame and simulate it in real-time.

    Args:
        history: The states of each time step.
        time_inc: The time increments of the simulation.
    """
    i_frame = 0
    n_time_steps = len(history)

    pg.init()
    screen = pg.display.set_mode(RenderState.display_size)
    clock = pg.time.Clock()
    RenderState.is_running = True

    while RenderState.is_running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                RenderState.is_running = False
                return

        time_step = get_time_step(i_frame, RenderState.framerate, time_inc)
        if time_step >= n_time_steps:
            RenderState.is_running = False
            continue

        # TODO: Render
        screen.fill(RenderState.bg_color)
        pg.display.update()

        time_step += 1
        clock.tick(RenderState.framerate)
