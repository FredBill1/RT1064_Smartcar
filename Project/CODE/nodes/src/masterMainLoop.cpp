#include "masterMainLoop.hpp"

Task_t Init() {
    for (auto mag : magnets) mag.set(1);
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

        masterGlobalVars.send_rects_enabled(true, rectMaxDistError * rectMaxDistError);
        WAIT_MOVE_BASE_REACHED;
        masterGlobalVars.send_rects_enabled(false);

        masterGlobalVars.send_art_cur_index(cur);
        utils::sendArtSnapshotTask();
        if constexpr (use_art) GUARD_COND(utils::waitArtSnapshot());
    }
    return true;
}

Task_t carryRects(int& carrying_cnt) {
    // 动物：左
    // 交通工具：上
    // 水果：右
    using namespace ResultCatgory;

    int catgory_cnt[4]{0};
    for (int i = 0; i < magnet::cnt; ++i) ++catgory_cnt[int(masterGlobalVars.art_results[i])];

    MoveBase::State state;
    moveBase.get_state(state);

    if (catgory_cnt[int(Major::vehicle)]) {                                                     // 有交通工具
        if (catgory_cnt[int(Major::vehicle)] + catgory_cnt[int(Major::None)] == magnet::cnt) {  // 只有交通工具，上
            moveBase.send_goal(state.x(), fieldHeight + carryExtendPadding, PI_2);
            WAIT_MOVE_BASE_REACHED;
            carrying_cnt -= utils::dropCatgory(Major::vehicle);
        } else {
            float cur_pos[2]{float(state.x()), float(state.y())};
            constexpr float POS[4][2]{
                // 左上
                {carrySidePadding, fieldHeight + carryExtendPadding},   // 上，交通工具
                {-carryExtendPadding, fieldHeight - carrySidePadding},  // 左，动物
                // 右上
                {fieldWidth - carrySidePadding, fieldHeight + carryExtendPadding},  // 上，交通工具
                {fieldWidth + carryExtendPadding, fieldHeight - carrySidePadding},  // 右，水果
            };
            constexpr Major CATGORY[4]{Major::vehicle, Major::animal, Major::vehicle, Major::fruit};
            int idx = int(catgory_cnt[int(Major::animal)] < catgory_cnt[int(Major::fruit)] ||
                          (catgory_cnt[int(Major::animal)] == catgory_cnt[int(Major::fruit)] && cur_pos[0] > (fieldWidth / 2)))
                      << 1;
            idx |= utils::calcDist(cur_pos, POS[idx]) < utils::calcDist(cur_pos, POS[idx | 1]);

            moveBase.send_goal(POS[idx][0], POS[idx][1], std::atan2(POS[idx][1] - cur_pos[1], POS[idx][0] - cur_pos[0]));
            WAIT_MOVE_BASE_REACHED;
            carrying_cnt -= utils::dropCatgory(CATGORY[idx]);

            moveBase.send_goal(POS[idx ^ 1][0], POS[idx ^ 1][1],
                               std::atan2(POS[idx ^ 1][1] - POS[idx][1], POS[idx ^ 1][0] - POS[idx][0]));
            WAIT_MOVE_BASE_REACHED;
            carrying_cnt -= utils::dropCatgory(CATGORY[idx ^ 1]);
        }
    } else {  // 没有交通工具
        constexpr float POS[2][2]{{-carryExtendPadding, PI}, {fieldWidth + carryExtendPadding, 0}};
        constexpr Major CATGORY[2]{Major::animal, Major::fruit};
        int idx = catgory_cnt[int(Major::animal)] < catgory_cnt[int(Major::fruit)] ||
                  (catgory_cnt[int(Major::animal)] == catgory_cnt[int(Major::fruit)] && state.x() > (fieldWidth / 2));
        moveBase.send_goal(POS[idx][0], state.y(), POS[idx][1]);
        WAIT_MOVE_BASE_REACHED;
        carrying_cnt -= utils::dropCatgory(CATGORY[idx]);
    }
    return true;
}

Task_t finalCarry() {
    // 动物：左
    // 交通工具：上
    // 水果：右
    using namespace ResultCatgory;
    float _targets[4][2];
    float* cur_pos = _targets[0];
    float(*targets)[2] = _targets + 1;
    Major catgory[3];
    MoveBase::State state;
    moveBase.get_state(state);
    cur_pos[0] = state.x(), cur_pos[1] = state.y();

    int catgory_cnt[4]{0};
    for (int i = 0; i < magnet::cnt; ++i) ++catgory_cnt[int(masterGlobalVars.art_results[i])];

    if (catgory_cnt[int(Major::vehicle)] && catgory_cnt[int(Major::fruit)]) {  // 右上
        float dx = 2 * fieldWidth - cur_pos[0], dy = 2 * fieldHeight - cur_pos[1];
        float x = cur_pos[0] + dx * (fieldHeight - cur_pos[1]) / dy, y = cur_pos[1] + dy * (fieldWidth - cur_pos[0]) / dx;
        int idx = x > fieldWidth;
        idx ? (x = 2 * fieldWidth - x) : (y = 2 * fieldHeight - y);
        targets[idx][0] = std::min(x, fieldWidth - carryExtendPadding), targets[idx][1] = fieldHeight + carryExtendPadding;
        targets[idx ^ 1][0] = fieldWidth + carryExtendPadding, targets[idx ^ 1][1] = std::min(y, fieldHeight - carrySidePadding);
        catgory[idx] = Major::vehicle, catgory[idx ^ 1] = Major::fruit;
    } else if (catgory_cnt[int(Major::vehicle)]) {  // 上
        float x = cur_pos[0] - cur_pos[0] * (fieldHeight - cur_pos[1]) / (2 * fieldHeight - cur_pos[1]);
        targets[0][0] = std::max(x, carrySidePadding), targets[0][1] = fieldHeight + carryExtendPadding;
        catgory[0] = Major::vehicle;
    } else if (catgory_cnt[int(Major::fruit)]) {  // 右
        float y = cur_pos[1] - cur_pos[1] * (fieldWidth - cur_pos[0]) / (2 * fieldWidth - cur_pos[0]);
        targets[0][0] = fieldWidth + carryExtendPadding, targets[0][1] = std::max(y, carrySidePadding);
        catgory[0] = Major::fruit;
    }

    int target_cnt = bool(catgory_cnt[int(Major::vehicle)]) + bool(catgory_cnt[int(Major::fruit)]);
    if (catgory_cnt[int(Major::animal)]) {  // 左
        targets[target_cnt][0] = -carryExtendPadding, targets[target_cnt][1] = carrySidePadding;
        catgory[target_cnt] = Major::animal;
        ++target_cnt;
    }
    for (int i = 0; i < target_cnt; ++i) {
        moveBase.send_goal(targets[i][0], targets[i][1],
                           std::atan2(targets[i][1] - targets[i - 1][1], targets[i][0] - targets[i - 1][0]));
        WAIT_MOVE_BASE_REACHED;
        utils::dropCatgory(catgory[i]);
    }
    return true;
}

Task_t MainProcess() {
    SHOW_STATE("MAIN");

    MoveBase::State state;
    int carrying_cnt = 0;
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
        masterGlobalVars.send_rects_enabled(true, rectMaxDistError * rectMaxDistError);
        WAIT_MOVE_BASE_REACHED;
        masterGlobalVars.send_rects_enabled(false);

        // 发送art拍照指令
        int magnet_index = utils::find_idle_magnet_index();
        masterGlobalVars.send_art_cur_index(magnet_index);
        utils::sendArtSnapshotTask();
        if constexpr (use_art) GUARD_COND(utils::waitArtSnapshot());

        // 计算拾取卡片的位姿
        moveBase.get_state(state);
        pose_kalman::T pose_xy[2]{state.x(), state.y()};
        pose_kalman::T target_pos[3];
        pose_kalman::magnetAlign(pose_xy, coords[cur_target], magnet_index, target_pos);

        // 发布拾取卡片位姿的指令
        moveBase.send_goal(target_pos[0], target_pos[1], target_pos[2]);
        WAIT_MOVE_BASE_REACHED;

        // 拾取卡片
        auto& srv = (magnet_index & 1 ? srv_r : srv_l);
        srv.max();
        rt_thread_mdelay(grab_srv_down_delay_ms);
        srv.min();

        // 删去当前目标点
        masterGlobalVars.coord_valid.reset(cur_target);

        if (rect_index == coords_cnt) break;
        if (++carrying_cnt == magnet::cnt) {
            if constexpr (!use_art) {
                carrying_cnt = 0;
                continue;
            }
            WAIT_FOR(masterGlobalVars.wait_art_result(mainloop_timeout));
            RUN_TASK(carryRects(carrying_cnt));
        }
    }
    if constexpr (!use_art) return true;
    if (carrying_cnt) {
        WAIT_FOR(masterGlobalVars.wait_art_result(mainloop_timeout));
        RUN_TASK(finalCarry());
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

Task_t LoopIter() {
    RUN_TASK(Reset());
    RUN_TASK(GetCoords());
    // RUN_TASK(SolveFirstTSP());
    RUN_TASK(ResetPos());
    // RUN_TASK(TraverseAndDetect());
    RUN_TASK(MainProcess());

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

bool masterMainLoopNode() { return FuncThread(masterMainLoopEntry, "masterMainLoop", 4096, Thread::lowest_priority); }