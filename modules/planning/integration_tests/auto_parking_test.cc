
#include "cyber/time/clock.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"

namespace apollo {

namespace planning {

DECLARE_string(test_localization_file);
DECLARE_string(test_chassis_file);

/**
 * @class AutoParkingTest
 * @brief This is an integration test for auto parking function
 */

class AutoParkingTest : public PlanningTestBase {
    public:
    virtual void SetUp() {
        FLAGS_use_multi_thread_to_add_obstacles = false;
        FLAGS_enable_multi_thread_in_dp_st_graph = false;
        FLAGS_use_navigation_mode = false;
        FLAGS_test_data_dir = "/apollo/modules/planning/testdata/garage_test";
        FLAGS_planning_upper_speed_limit = 12.5;
        FLAGS_test_previous_planning_file = "";
        FLAGS_test_localization_file = "";
        FLAGS_test_chassis_file = "";
        FLAGS_enable_rss_info = false;

        FLAGS_enable_scenario_stop_sign = false;
        FLAGS_enable_scenario_traffic_light = false;
    }
};

TEST_F(AutoParkingTest, simple_parking_01) {
    FLAGS_test_localization_file = "stop_dest_localization.pb.txt";
    FLAGS_test_chassis_file = "stop_dest_chassis.pb.txt";
    PlanningTestBase::SetUp();

    RUN_GOLDEN_TEST(0);
}

}  // namespace planning
}  // namespace apollo

TMAIN;