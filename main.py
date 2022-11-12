from src import simulation, rendering


def main():
    sim_constants = simulation.SimConstants(
        time_inc=0.01, n_individuals=100, individual_radius=1,
        collision_rebound=10, mass=10, max_pos=([50, 50]), n_time_steps=1000
    )
    render_state = rendering.RenderState(display_size=(800, 800), individual_color='#000000', bg_color='#C336C9', framerate=20)
    random_seed = None


    history = simulation.run_simulation(sim_constants=sim_constants, seed=random_seed)
    rendering.render_simulation(history=history, render_state=render_state, sim_constants=sim_constants)


if __name__ == "__main__":
    main()