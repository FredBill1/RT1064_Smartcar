#include "utils/FuncThread.hpp"
//
#include "devices.hpp"

static void testArmDrvEntry() {
    armDrv.initial_pose();
    for (;;) {
        for (int i = 0; i < 3; ++i) {
            ips.printf("pick %d\n", i);
            while (!master_key[0].pressing()) rt_thread_mdelay(100);
            ips.printf("pick ");
            armDrv.pick();
            ips.printf("before_place ");
            armDrv.before_place();
            ips.printf("place\n");
            armDrv.place(i);
            ips.printf("initial_pose\n");
        }
        for (int i = 0; i < 3; ++i) {
            ips.printf("drop %d\n", i);
            while (!master_key[0].pressing()) rt_thread_mdelay(100);
            ips.printf("drop\n");
            armDrv.drop(i);
        }
    }
}

bool testArmDrvNode() { return FuncThread(testArmDrvEntry, "testArmDrv"); }