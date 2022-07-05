#include "masterMainLoop.hpp"

Task_t Init() {
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
    WAIT_FOR(masterGlobalVars.wait_for_coord_recv(mainloop_timeout));
    utils::sendSlaveTask(SlaveGlobalVars::RECT);
    masterGlobalVars.get_coord_recv();
    utils::sendCoords();
    draw_corr(coords + 1, coords_cnt, borderWidth, borderHeight);
    return true;
}

Task_t SolveFirstTSP() {
    SHOW_STATE("TSP1");
    tsp.N = coords_cnt + 1;
    coords[0][0] = initial_position[0], coords[0][1] = initial_position[1];
    for (int i = 0; i <= coords_cnt; ++i) {
        tsp.dist[i][i] = 0;
        for (int j = i + 1; j <= coords_cnt; ++j) tsp.dist[i][j] = tsp.dist[j][i] = utils::calcDist(coords[i], coords[j]);
    }
    tsp.solve_without_returning();

    for (int i = 1; i <= coords_cnt; ++i) {
        int u = tsp.hamilton_path[i - 1], v = tsp.hamilton_path[i];
        drawLine(coords[u][0] * tsp_k, coords[u][1] * tsp_k, coords[v][0] * tsp_k, coords[v][1] * tsp_k,
                 [](int x, int y) { ips114_drawpoint(x, N / 4 - y, ips.Red); });
    }
    return true;
}

Task_t ResetPos() {
    MoveBase::State state(systick.get_us(), initial_position[0], initial_position[1], initial_position[2], 0, 0, 0);
    moveBase.send_set_state(state);
    return true;
}

Task_t TraverseAndDetect() {
    SHOW_STATE("TRAV");
    for (int i = 1; i < tsp.N; ++i) {
        MoveBase::State state;
        moveBase.get_state(state);
        int cur = tsp.hamilton_path[i];
        float x = coords[cur][0], y = coords[cur][1], yaw = std::atan2(y - state.y(), x - state.x());
        moveBase.send_goal(x - art_cam_dist * std::cos(yaw), y - art_cam_dist * std::sin(yaw), yaw);

        masterGlobalVars.send_rects_enabled(true, rectMaxDistErrorSquared);
        GUARD_COND(utils::moveBaseReachedCheck());
        masterGlobalVars.send_rects_enabled(false);

        masterGlobalVars.send_art_cur_index(cur);
        utils::sendArtSnapshotTask();
        if constexpr (use_art) GUARD_COND(utils::waitArtSnapshot());
    }
    return true;
}

Task_t MainProcess() {
    SHOW_STATE("MAIN");

    MoveBase::State state;
    int magnet_index = 0;
    for (int rect_index = 1; rect_index <= coords_cnt; ++rect_index) {
        // 获取距离最近的卡片作为目标点
        moveBase.get_state(state);
        int cur_target;
        {
            float cur_pos[2]{(float)state.x(), (float)state.y()};
            float min_dist = std::numeric_limits<float>::infinity();
            for (int i = 1; i <= coords_cnt; ++i)
                if (masterGlobalVars.coord_valid.test(i))
                    if (float dist = utils::calcDist(cur_pos, coords[i]); dist < min_dist) min_dist = dist, cur_target = i;
        }

        // 发布目标位置指令
        float x = coords[cur_target][0], y = coords[cur_target][1], yaw = std::atan2(y - state.y(), x - state.x());
        moveBase.send_goal(x - art_cam_dist * std::cos(yaw), y - art_cam_dist * std::sin(yaw), yaw);

        // 导航到目标位置
        masterGlobalVars.send_rects_enabled(true, rectMaxDistErrorSquared);
        GUARD_COND(utils::moveBaseReachedCheck());
        masterGlobalVars.send_rects_enabled(false);

        // 发送art拍照指令
        masterGlobalVars.send_art_cur_index(cur_target);
        utils::sendArtSnapshotTask();
        if constexpr (use_art) GUARD_COND(utils::waitArtSnapshot());

        // 计算拾取卡片的位姿
        moveBase.get_state(state);
        pose_kalman::T pose_xy[2]{state.x(), state.y()};
        pose_kalman::T target_pos[3];
        pose_kalman::magnetAlign(pose_xy, coords[cur_target], magnet_index, target_pos);

        // 发布拾取卡片位姿的指令
        moveBase.send_goal(target_pos[0], target_pos[1], target_pos[2]);
        GUARD_COND(utils::moveBaseReachedCheck());

        // 拾取卡片
        auto& srv = (magnet_index & 1 ? srv_r : srv_l);
        srv.max();
        magnets[magnet_index].set(true);
        rt_thread_mdelay(grab_srv_down_delay_ms);
        srv.min();

        // 删去当前目标点
        masterGlobalVars.coord_valid.reset(cur_target);
        if (++magnet_index == 6) return true;
    }
    return true;
}

static inline void Idle() {
    SHOW_STATE("IDLE");
    for (;;) {
        keyScan();
        if (key_pressing[4]) break;
        if (key_pressing[1]) utils::sendSlaveTask(SlaveGlobalVars::A4_PREPARE);
        if (key_pressing[2]) utils::sendSlaveTask(SlaveGlobalVars::RECT);
        if (key_pressing[3]) utils::sendArtSnapshotTask();
    }
    masterGlobalVars.reset_requested();  // clear the reset flag
}

static inline void LoopIter() {
    RUN_TASK(Reset);
    RUN_TASK(GetCoords);
    // RUN_TASK(SolveFirstTSP);
    RUN_TASK(ResetPos);
    // RUN_TASK(TraverseAndDetect);
    RUN_TASK(MainProcess);
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

bool masterMainLoopNode() { return FuncThread(masterMainLoopEntry, "masterMainLoop", 4096, Thread::lowest_priority); }