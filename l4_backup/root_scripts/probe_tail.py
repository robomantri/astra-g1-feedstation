    obs, _ = env.get_observations()
    uenv = env.unwrapped
    scene = uenv.scene
    cube = scene["cube"]
    table = scene["table"]
    robot = scene["robot"]
    palm_i = robot.body_names.index("right_palm_link")

    def v(t):
        return tuple(round(float(x), 3) for x in t)

    print("[P] cube  init xyz =", v(cube.data.root_pos_w[0, :3]))
    print("[P] table init xyz =", v(table.data.root_pos_w[0, :3]))
    print("[P] robot init xyz =", v(robot.data.root_pos_w[0, :3]))
    print("[P] robot init quat=", v(robot.data.root_quat_w[0, :4]))
    print("[P] env origin     =", v(scene.env_origins[0]))
    print("[P] palm  init xyz =", v(robot.data.body_pos_w[0, palm_i, :3]))
    print("[P] body_names[:6] =", robot.body_names[:6])

    tm = uenv.termination_manager
    print("[P] termination terms:", list(tm.active_terms))

    cz0 = float(cube.data.root_pos_w[0, 2])
    steps = args_cli.max_steps or 600
    ep = 0
    best = {"lift": -9.9, "mindist": 9.9}
    for step in range(1, steps + 1):
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
        c = cube.data.root_pos_w[0, :3]
        p = robot.data.body_pos_w[0, palm_i, :3]
        r = robot.data.root_pos_w[0, :3]
        d = float(torch.linalg.norm(p - c))
        lift = float(c[2]) - cz0
        best["lift"] = max(best["lift"], lift)
        best["mindist"] = min(best["mindist"], d)
        if step % 25 == 0:
            print(f"[P] s{step:4d} root={v(r)} cube={v(c)} palm={v(p)} "
                  f"palm-cube={d:.3f} lift={lift:+.3f}")
        if bool(dones[0]):
            fired = [n for n in tm.active_terms
                     if bool(tm.get_term(n)[0])]
            ep += 1
            print(f"[P] === EPISODE {ep} ended at step {step}: {fired} "
                  f"| best_lift={best['lift']:+.3f} min_palm_cube={best['mindist']:.3f}")
            best = {"lift": -9.9, "mindist": 9.9}
            cz0 = float(cube.data.root_pos_w[0, 2])
    print("[P] done")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
