    obs, _ = env.get_observations()
    uenv = env.unwrapped
    scene = uenv.scene
    cube = scene["cube"]
    robot = scene["robot"]
    device = robot.device
    palm_i = robot.body_names.index("right_palm_link")
    left_palm_i = robot.body_names.index("left_palm_link")

    ATTACH_DIST = float(os.environ.get("ATTACH_DIST", "0.30"))
    _box_size = os.environ.get("BOX_SIZE", "0.04,0.04,0.04")
    _box_mass = os.environ.get("BOX_MASS", "0.2")
    print(f"[W] object size={_box_size} mass={_box_mass} ATTACH_DIST={ATTACH_DIST} (bimanual midpoint weld)")

    attached = torch.zeros(uenv.num_envs, dtype=torch.bool, device=device)

    steps = args_cli.max_steps or 600
    n_attach_events = 0
    for step in range(1, steps + 1):
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)

            r_palm_pos = robot.data.body_pos_w[:, palm_i, :3].clone()
            l_palm_pos = robot.data.body_pos_w[:, left_palm_i, :3].clone()
            base_quat = robot.data.root_quat_w[:, :4].clone()
            cube_pos = cube.data.root_pos_w[:, :3].clone()

            dist = torch.linalg.norm(r_palm_pos - cube_pos, dim=-1)
            newly_attach = (~attached) & (dist < ATTACH_DIST) & (~dones)
            if bool(newly_attach.any()):
                idx = newly_attach.nonzero(as_tuple=True)[0]
                attached[idx] = True
                n_attach_events += int(idx.numel())
                for e in idx.tolist():
                    print(f"[W] step {step:4d} env{e} ATTACHED  dist={float(dist[e]):.3f}  "
                          f"l_palm={l_palm_pos[e].tolist()}  r_palm={r_palm_pos[e].tolist()}")

            if bool(attached.any()):
                idx = attached.nonzero(as_tuple=True)[0]
                # bimanual weld: pose recomputed from BOTH current palms every frame,
                # not latched to one hand -- so the crate always sits between them.
                mid_pos = 0.5 * (l_palm_pos[idx] + r_palm_pos[idx])
                pose = torch.cat([mid_pos, base_quat[idx]], dim=-1)
                cube.write_root_pose_to_sim(pose, env_ids=idx)
                zero_vel = torch.zeros(idx.numel(), 6, device=device)
                cube.write_root_velocity_to_sim(zero_vel, env_ids=idx)

            if bool(dones.any()):
                idx = dones.nonzero(as_tuple=True)[0]
                attached[idx] = False

        if step % 25 == 0:
            print(f"[W] s{step:4d} attached={int(attached.sum())}/{uenv.num_envs} "
                  f"min_dist={float(dist.min()):.3f}")

    print(f"[W] done. total attach events={n_attach_events}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
