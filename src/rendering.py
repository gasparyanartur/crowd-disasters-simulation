"""Functionality related to rendering the finished simulation."""
from __future__ import annotations
from typing import TYPE_CHECKING

from dataclasses import dataclass

import pygame as pg

if TYPE_CHECKING:
    from state import State
    from simulation import SimConstants


@dataclass(init=True, slots=True)
class RenderState:
    display_size: tuple[int, int] = 800, 800
    bg_color: str = "#C336C9"
    is_running: bool = False
    is_paused: bool = False
    framerate: int = 20


def get_time_step(i_frame: int, frame_rate: int, time_inc: float) -> int:
    """Get the time step corresponding to actual time passed of simulation.

    Example:
        50 fps -> 0.02s per frame
        0.01 time_inc -> 2 time steps per frame
        3 frames -> 6 time_steps

    Args:
        i_frame: The frame currently being rendered:
        frame_rate: Number of frames rendered per second.
        time_inc: The time passed between time steps.
    """
    return (i_frame / frame_rate) // time_inc


def render_simulation(history: list[State], render_state: RenderState, sim_constants: SimConstants) -> None:
    """Given a finished simulation, run over each frame and simulate it in real-time.

    Args:
        history: The states of each time step.
        render_state. The settings of the rendering.
        sim_constants: The settings of the simulation.
    """
    i_frame = 0
    n_time_steps = len(history)

    pg.init()
    screen = pg.display.set_mode(render_state.display_size)
    clock = pg.time.Clock()
    render_state.is_running = True

    while render_state.is_running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                render_state.is_running = False
                return

        time_step = get_time_step(i_frame, render_state.framerate, sim_constants.time_inc)
        if time_step >= n_time_steps:
            render_state.is_running = False
            continue

        # TODO: Render
        screen.fill(render_state.bg_color)
        pg.display.update()

        time_step += 1
        clock.tick(render_state.framerate)
