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
            masterGlobalVars.get_coord_recv();
            draw_corr(coords + 1, coords_cnt, borderWidth, borderHeight);
        }
    }
    utils::sendSlaveTask(SlaveGlobalVars::RECT);
    utils::sendCoords();
    return true;
}

Task_t ResetPos() {
    MoveBase::State state(systick.get_us(), initial_position[0], initial_position[1], initial_position[2], 0, 0, 0);
    moveBase.send_set_state(state);
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
    MoveBase::Goal goal_carry = GOAL_CARRY;

    if (catgory_cnt[int(Major::vehicle)]) {                                                     // 有交通工具
        if (catgory_cnt[int(Major::vehicle)] + catgory_cnt[int(Major::None)] == magnet::cnt) {  // 只有交通工具，上
            goal_carry.x = state.x();
            goal_carry.y = fieldHeight + carryExtendPadding;
            goal_carry.yaw = PI_2;
            moveBase.send_goal(goal_carry);
            WAIT_MOVE_BASE_GOAL_REACHED;
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

            goal_carry.x = POS[idx][0];
            goal_carry.y = POS[idx][1];
            goal_carry.yaw = std::atan2(POS[idx][1] - cur_pos[1], POS[idx][0] - cur_pos[0]);
            moveBase.send_goal(goal_carry);
            WAIT_MOVE_BASE_GOAL_REACHED;
            carrying_cnt -= utils::dropCatgory(CATGORY[idx]);

            goal_carry.x = POS[idx ^ 1][0];
            goal_carry.y = POS[idx ^ 1][1];
            goal_carry.yaw = std::atan2(POS[idx ^ 1][1] - POS[idx][1], POS[idx ^ 1][0] - POS[idx][0]);
            moveBase.send_goal(goal_carry);

            WAIT_MOVE_BASE_GOAL_REACHED;
            carrying_cnt -= utils::dropCatgory(CATGORY[idx ^ 1]);
        }
    } else {  // 没有交通工具
        constexpr float POS[2][2]{{-carryExtendPadding, PI}, {fieldWidth + carryExtendPadding, 0}};
        constexpr Major CATGORY[2]{Major::animal, Major::fruit};
        int idx = catgory_cnt[int(Major::animal)] < catgory_cnt[int(Major::fruit)] ||
                  (catgory_cnt[int(Major::animal)] == catgory_cnt[int(Major::fruit)] && state.x() > (fieldWidth / 2));
        goal_carry.x = POS[idx][0];
        goal_carry.y = state.y();
        goal_carry.yaw = POS[idx][1];
        moveBase.send_goal(goal_carry);
        WAIT_MOVE_BASE_GOAL_REACHED;
        carrying_cnt -= utils::dropCatgory(CATGORY[idx]);
    }
    return true;
}

Task_t carryRects_only1(int& carrying_cnt, float* min_dist2_thresh = nullptr, bool* dist_result = nullptr) {
    // 动物：左
    // 交通工具：上
    // 水果：右
    using namespace ResultCatgory;
    using std::max, std::min;

    int catgory_cnt[4]{0};
    for (int i = 0; i < magnet::cnt; ++i) ++catgory_cnt[int(masterGlobalVars.art_results[i])];

    MoveBase::State state;
    moveBase.get_state(state);
    MoveBase::Goal goal_carry = GOAL_CARRY;

    float CUR_POS[2]{float(state.x()), float(state.y())};
    float X = max(min(CUR_POS[0], fieldWidth - carrySidePadding), carrySidePadding);
    float Y = max(min(CUR_POS[1], fieldHeight - carrySidePadding), carrySidePadding);
    float POS[][2]{
        {-carryExtendPadding, Y},               // 左，动物
        {X, fieldHeight + carryExtendPadding},  // 上，交通工具
        {fieldWidth + carryExtendPadding, Y},   // 右，水果
    };
    constexpr pose_kalman::T YAW[3]{0, -PI_2, PI};
    constexpr Major CATGORY[3]{Major::animal, Major::vehicle, Major::fruit};
    float min_dist2 = std::numeric_limits<float>::infinity();
    int idx = 0;
    for (int i = 0; i < 3; ++i)
        if (catgory_cnt[int(CATGORY[i])])
            if (float dist2 = utils::calcDist2(CUR_POS, POS[i]); dist2 < min_dist2) min_dist2 = dist2, idx = i;

    if (min_dist2_thresh) {
        if (min_dist2 >= *min_dist2_thresh) {
            *dist_result = false;
            return true;
        } else
            *dist_result = true;
    }

    goal_carry.x = POS[idx][0];
    goal_carry.y = POS[idx][1];
    goal_carry.yaw = YAW[idx];
    moveBase.send_goal(goal_carry);
    // WAIT_MOVE_BASE_GOAL_REACHED;
    WAIT_FOR(moveBase.wait_xy_near(mainloop_timeout));
    carrying_cnt -= utils::dropCatgory(CATGORY[idx]);

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

    MoveBase::Goal goal_carry = GOAL_CARRY;
    for (int i = 0; i < target_cnt; ++i) {
        goal_carry.x = targets[i][0], goal_carry.y = targets[i][1];
        goal_carry.yaw = std::atan2(targets[i - 1][1] - targets[i][1], targets[i - 1][0] - targets[i][0]);
        moveBase.send_goal(goal_carry);
        // WAIT_MOVE_BASE_GOAL_REACHED;
        WAIT_FOR(moveBase.wait_xy_near(mainloop_timeout));
        utils::dropCatgory(catgory[i]);
    }
    return true;
}

Task_t MainProcess() {
    SHOW_STATE("MAIN");

    MoveBase::State state;
    int carrying_cnt = 0;
    MoveBase::Goal goal_navi = GOAL_NAVI;
    MoveBase::Goal goal_pick = GOAL_PICK;

    for (int rect_index = 1; rect_index <= coords_cnt; ++rect_index) {
        // 获取距离最近的卡片作为目标点
        int cur_target;
        for (;;) {
            moveBase.get_state(state);
            float cur_pos[2]{(float)state.x(), (float)state.y()};
            float min_dist2 = std::numeric_limits<float>::infinity();
            for (int i = 1; i <= coords_cnt; ++i)
                if (masterGlobalVars.coord_valid.test(i))
                    if (float dist2 = utils::calcDist2(cur_pos, coords[i]); dist2 < min_dist2) min_dist2 = dist2, cur_target = i;

            if (carrying_cnt + (coords_cnt - rect_index + 1) < magnet::cnt || carrying_cnt < 3) break;
            bool carried;
            RUN_TASK(carryRects_only1(carrying_cnt, &min_dist2, &carried));
            if (!carried) break;
        }

        // 发布目标位置指令
        float x = coords[cur_target][0], y = coords[cur_target][1], yaw = std::atan2(y - state.y(), x - state.x());
        goal_navi.x = x - art_cam_dist * std::cos(yaw);
        goal_navi.y = y - art_cam_dist * std::sin(yaw);
        goal_navi.yaw = yaw;
        moveBase.send_goal(goal_navi);

        // 导航到目标位置
        masterGlobalVars.send_rects_enabled(true, rectMaxDistError * rectMaxDistError);
        WAIT_MOVE_BASE_GOAL_REACHED;
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
        goal_pick.x = target_pos[0];
        goal_pick.y = target_pos[1];
        goal_pick.yaw = target_pos[2];
        moveBase.send_goal(goal_pick);
        WAIT_MOVE_BASE_GOAL_REACHED;

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
            // RUN_TASK(carryRects(carrying_cnt));
            RUN_TASK(carryRects_only1(carrying_cnt));
        }
    }
    if constexpr (!use_art) return true;
    if (carrying_cnt) {
        WAIT_FOR(masterGlobalVars.wait_art_result(mainloop_timeout));
        RUN_TASK(finalCarry());
    }
    return true;
}

Task_t ReturnGarage() {
    MoveBase::State state;
    moveBase.get_state(state);

    MoveBase::Goal goal = GOAL_CARRY;
    goal.x = garage_position1[0];
    goal.y = garage_position1[1];
    goal.yaw = std::atan2(garage_position1[1] - state.y(), garage_position1[0] - state.x());
    moveBase.send_goal(goal);
    WAIT_MOVE_BASE_GOAL_REACHED;

    goal.x = garage_position2[0];
    goal.y = garage_position2[1];
    goal.yaw = -PI_2;
    moveBase.send_goal(goal);
    WAIT_MOVE_BASE_GOAL_REACHED;
    return true;
}

static inline void Idle() {
    SHOW_STATE("IDLE");
    for (;;) {
        keyScan();
        if (key_pressing[4]) break;
        if (key_pressing[1]) utils::sendSlaveTask(SlaveGlobalVars::A4);
        if (key_pressing[2]) utils::sendSlaveTask(SlaveGlobalVars::RECT);
        if (key_pressing[3]) utils::sendArtSnapshotTask();
        rt_thread_mdelay(300);
    }
    masterGlobalVars.reset_requested();  // clear the reset flag
}

Task_t LoopIter() {
    RUN_TASK(Reset());
    RUN_TASK(GetCoords());
    RUN_TASK(ResetPos());
    RUN_TASK(MainProcess());
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

bool masterMainLoopNode() { return FuncThread(masterMainLoopEntry, "masterMainLoop", 4096, Thread::lowest_priority); }