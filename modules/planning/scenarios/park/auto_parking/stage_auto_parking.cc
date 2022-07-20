
#include "modules/planning/scenarios/park/auto_parking/stage_auto_parking.h"

namespace apollo {

namespace planning {

namespace scenario {

namespace auto_parking{

Stage::StageStatus StageAutoParking::Process(const common::TrajectoryPoint& planning_init_point,
                                             Frame* frame) {
    (void) planning_init_point;
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
    bool plan_ok = ExecuteTaskOnOpenSpace(frame);
    if (!plan_ok) {
        AERROR << "StageAutoParking planning error";
        return StageStatus::ERROR;
    }
    return StageStatus::RUNNING;
}

Stage::StageStatus StageAutoParking::FinishStage() { return Stage::FINISHED; }

}  // namespace auto_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo