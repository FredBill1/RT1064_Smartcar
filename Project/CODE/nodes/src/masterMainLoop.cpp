#include "masterMainLoop.hpp"

Task_t Init() {
    for (auto& mag : magnets) mag.set(1);
    utils::clear_screen();
    return true;
}

Task_t Reset() {
    masterGlobalVars.reset_states();
    utils::sendSlaveTask(SlaveGlobalVars::RESET);

    utils::clear_screen();
    return true;
}

Task_t GetCoords() {
    SHOW_STATE("GETC");
    for (;;) {
        CHECK_RESET_SIGNAL();
        keyScan();
        if (key_pressing[3]) break;
        if (masterGlobalVars.wait_for_coord_recv(100)) {
            draw_corr(coords + 1, coords_cnt, borderWidth, borderHeight, WHITE);  // 清空上一次画的
            masterGlobalVars.get_coord_recv();
            draw_corr(coords + 1, coords_cnt, borderWidth, borderHeight);
        }
    }
    utils::sendSlaveTask(SlaveGlobalVars::RECT);
    utils::sendCoords();
    return true;
}

Task_t SolveTSP() {
    SHOW_STATE("TSP ");
    tsp.N = coords_cnt + 3;  // 起点、终点、辅助点
    constexpr float start_coord[2]{0, 0}, end_coord[2]{fieldWidth * 2, fieldHeight * 2};

    tsp.dist[0][coords_cnt + 1] = tsp.dist[coords_cnt + 1][0] = std::numeric_limits<float>::infinity();  // 起点、终点
    tsp.dist[0][coords_cnt + 2] = tsp.dist[coords_cnt + 2][0] = 0;                                       // 起点、辅助点
    tsp.dist[coords_cnt + 1][coords_cnt + 2] = tsp.dist[coords_cnt + 2][coords_cnt + 1] = 0;             // 终点、辅助点
    for (int i = 1; i <= coords_cnt; ++i) {
        tsp.dist[i][i] = 0;
        tsp.dist[i][0] = tsp.dist[0][i] = utils::calcDist(coords[i], start_coord);                           // 起点
        tsp.dist[i][coords_cnt + 1] = tsp.dist[coords_cnt + 1][i] = utils::calcDist(coords[i], end_coord);   // 终点
        tsp.dist[i][coords_cnt + 2] = tsp.dist[coords_cnt + 2][i] = std::numeric_limits<float>::infinity();  // 辅助点
        for (int j = i + 1; j <= coords_cnt; ++j) tsp.dist[i][j] = tsp.dist[j][i] = utils::calcDist(coords[i], coords[j]);
    }
    tsp.solve();

    // 把辅助点扔到最后
    if (tsp.hamilton_path[1] == coords_cnt + 2) std::reverse(tsp.hamilton_path + 1, tsp.hamilton_path + coords_cnt + 3);

    constexpr auto plot = [](int x, int y) { ips114_drawpoint(x, N / 4 - y, ips.Red); };
    drawLine(0, 0, coords[tsp.hamilton_path[1]][0] * tsp_k, coords[tsp.hamilton_path[1]][1] * tsp_k, plot);
    for (int i = 2; i <= coords_cnt; ++i) {
        int u = tsp.hamilton_path[i - 1], v = tsp.hamilton_path[i];
        drawLine(coords[u][0] * tsp_k, coords[u][1] * tsp_k, coords[v][0] * tsp_k, coords[v][1] * tsp_k, plot);
    }

    // 计算搬运顺序
    CarryOrder::calc();

    return true;
}

Task_t ResetPos() {
    MoveBase::State state(systick.get_us(), initial_position[0], initial_position[1], initial_position[2], 0, 0, 0);
    moveBase.send_set_state(state);
    return true;
}

Task_t OutGarage() {
    MoveBase::Goal goal_out_garage = GOAL_OUT_GARAGE;

    goal_out_garage.x = initial_position[0];
    goal_out_garage.y = out_garage_y;
    goal_out_garage.yaw = initial_position[2];
    moveBase.send_goal(goal_out_garage);
    WAIT_MOVE_BASE_GOAL_NEAR;

    return true;
}

Task_t MainProcess() {
    SHOW_STATE("MAIN");

    MoveBase::State state;

    MoveBase::Goal goal_navi_turn = GOAL_NAVI_TURN;
    MoveBase::Goal goal_navi_move = GOAL_NAVI_MOVE;
    MoveBase::Goal goal_navi_refine = GOAL_NAVI_REFINE;

    for (int target_index = 1; target_index <= coords_cnt; ++target_index) {
        int cur_target = tsp.hamilton_path[target_index];
        moveBase.get_state(state);
        pose_kalman::T x = coords[cur_target][0], y = coords[cur_target][1], yaw = std::atan2(y - state.y(), x - state.x());
        x -= art_cam_dist * std::cos(yaw);
        y -= art_cam_dist * std::sin(yaw);

        // 自转
        goal_navi_turn.x = state.x();
        goal_navi_turn.y = state.y();
        goal_navi_turn.yaw = yaw;
        moveBase.send_goal(goal_navi_turn);
        WAIT_MOVE_BASE_GOAL_NEAR;

        // 平移
        goal_navi_move.x = x;
        goal_navi_move.y = y;
        goal_navi_move.yaw = yaw;
        moveBase.send_goal(goal_navi_move);
        WAIT_MOVE_BASE_GOAL_NEAR;

        // 调整
        goal_navi_refine.x = x;
        goal_navi_refine.y = y;
        goal_navi_refine.yaw = yaw;
        moveBase.send_goal(goal_navi_refine);
        masterGlobalVars.send_rects_enabled(true, cur_target, rectMaxDistError * rectMaxDistError);
        WAIT_MOVE_BASE_GOAL_REACHED;
        masterGlobalVars.send_rects_enabled(false);

        // 发送art拍照指令
        rt_thread_mdelay(art_before_snapshot_delay);
        masterGlobalVars.send_art_cur_index(cur_target);
        utils::sendArtSnapshotTask();
        if constexpr (use_art) {
            GUARD_COND(utils::waitArtSnapshot());
        } else
            rt_thread_mdelay(500);

        // 机械臂抓取
        masterGlobalVars.wait_arm_initial_pose();
        armDrv.pick();
        masterGlobalVars.send_arm_picked();

        // 删除当前点
        masterGlobalVars.coord_valid.reset(cur_target);
    }

    return true;
}

Task_t Carry() {
    SHOW_STATE("CARY");
    using CarryOrder::catgory, CarryOrder::targets;
    using pose_kalman::T;

    MoveBase::State state;
    MoveBase::Goal goal_carry_turn = GOAL_CARRY_TURN;
    MoveBase::Goal goal_carry_move = GOAL_CARRY_MOVE;

    for (int i = 0; i < 3; ++i) {
        moveBase.get_state(state);
        T yaw = utils::wrapAngleNear(std::atan2(targets[i][1] - state.y(), targets[i][0] - state.x()), state.yaw());
        goal_carry_turn.x = state.x();
        goal_carry_turn.y = state.y();
        goal_carry_turn.yaw = yaw;
        moveBase.send_goal(goal_carry_turn);
        WAIT_MOVE_BASE_GOAL_NEAR;

        goal_carry_move.x = targets[i][0];
        goal_carry_move.y = targets[i][1];
        goal_carry_move.yaw = yaw;
        moveBase.send_goal(goal_carry_move);
        WAIT_MOVE_BASE_GOAL_NEAR;

        if (i == 0) WAIT_FOR(masterGlobalVars.wait_arm_placed(mainloop_timeout));

        armDrv.drop(i);
    }

    return true;
}

Task_t ReturnGarage() {
    SHOW_STATE("GARA");
    MoveBase::Goal goal_garage_turn = GOAL_GARAGE_TURN;
    MoveBase::Goal goal_garage_move = GOAL_GARAGE_MOVE;

    MoveBase::State state;

    // 转向左边
    moveBase.get_state(state);
    goal_garage_turn.x = state.x();
    goal_garage_turn.y = state.y();
    goal_garage_turn.yaw = PI;
    moveBase.send_goal(goal_garage_turn);
    WAIT_MOVE_BASE_GOAL_NEAR;

    // 通知art开始找边界
    utils::sendArtBorderTask();

    // 向右移动
    moveBase.get_state(state);
    goal_garage_move.x = 1e6;
    goal_garage_move.y = state.y();
    goal_garage_move.yaw = PI;
    moveBase.send_goal(goal_garage_move);
    masterGlobalVars.clear_art_border();
    WAIT_FOR(masterGlobalVars.wait_art_border(mainloop_timeout));

    // 向右微调
    goal_garage_move.x = state.x() + garage_left_padding;
    moveBase.send_goal(goal_garage_move);
    WAIT_MOVE_BASE_GOAL_NEAR;

    // 转向上边
    moveBase.get_state(state);
    goal_garage_turn.x = state.x();
    goal_garage_turn.y = state.y();
    goal_garage_turn.yaw = PI_2;
    moveBase.send_goal(goal_garage_turn);
    WAIT_MOVE_BASE_GOAL_NEAR;

    // 向下移动
    moveBase.get_state(state);
    goal_garage_move.x = state.x();
    goal_garage_move.y = -1e6;
    goal_garage_move.yaw = PI_2;
    moveBase.send_goal(goal_garage_move);
    rt_thread_mdelay(garage_down_delay);
    masterGlobalVars.clear_art_border();
    WAIT_FOR(masterGlobalVars.wait_art_border(mainloop_timeout));
    rt_thread_mdelay(garage_stop_delay);

    // 停车
    moveBase.get_state(state);
    goal_garage_move.x = state.x();
    goal_garage_move.y = state.y();
    goal_garage_move.yaw = state.yaw();
    moveBase.send_goal(goal_garage_move);
    WAIT_MOVE_BASE_GOAL_REACHED;

    return true;
}

static inline void Idle() {
    SHOW_STATE("IDLE");
    armDrv.reset();
    for (;;) {
        keyScan();
        if (key_pressing[4]) break;
        if (key_pressing[1]) utils::sendSlaveTask(SlaveGlobalVars::A4);
        if (key_pressing[2]) utils::sendSlaveTask(SlaveGlobalVars::RECT);
        if (key_pressing[3]) utils::sendArtSnapshotTask();
        if (key_pressing[0]) utils::sendArtBorderTask();
        ips114_showstr(188, 0, master_switch[0].get() ? "9dof" : "    ");
        ips114_showstr(188, 1, master_switch[0].get() ? "!imu" : "    ");

        rt_thread_mdelay(300);
    }
    masterGlobalVars.reset_requested();  // clear the reset flag
}

Task_t LoopIter() {
    RUN_TASK(Reset());
    RUN_TASK(GetCoords());
    RUN_TASK(SolveTSP());
    RUN_TASK(ResetPos());
    RUN_TASK(OutGarage());
    RUN_TASK(MainProcess());
    RUN_TASK(Carry());
    RUN_TASK(ReturnGarage());

    return true;
}

static inline void CleanUp() {}

static void masterMainLoopEntry() {
    Init();
    for (;;) {
        Idle();
        LoopIter();
        CleanUp();
    }
}

static void armControlEntry() {
    for (;;) {
        masterGlobalVars.wait_arm_picked();
        armDrv.before_place();
        ResultCatgory::Major catgory;
        if constexpr (use_art) {
            masterGlobalVars.wait_art_result();
            catgory = masterGlobalVars.art_last_result;
        } else {
            rt_thread_mdelay(800);
            catgory = ResultCatgory::Major(systick.get_us() % 3);
        }
        armDrv.place(CarryOrder::catgory_to_index[(int)catgory]);
        masterGlobalVars.send_arm_placed();
        armDrv.initial_pose();
        rt_thread_mdelay(arm_initial_pose_delay);
        masterGlobalVars.send_arm_initial_pose();
    }
}

bool masterMainLoopNode() {
    return FuncThread(armControlEntry, "armControl", 2048, Thread::lowest_priority - 1) &&
           FuncThread(masterMainLoopEntry, "masterMainLoop", 4096, Thread::lowest_priority);
}