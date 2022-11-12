import simulation
import rendering


def main():
    sim_constants = simulation.SimConstants(
        time_inc=0.01, n_individuals=100, individual_radius=1,
        collision_rebound=1, mass=10, max_pos=([50, 50]), n_time_steps=15
    )
    render_state = rendering.RenderState(display_size=(800, 800), bg_color='#C336C9', framerate=20)
    random_seed = 69420


    history = simulation.run_simulation(sim_constants=sim_constants, seed=random_seed)
    rendering.render_simulation(history=history, render_state=render_state, sim_constants=sim_constants.time_inc)


if __name__ == "__main__":
    main()