
#include "modules/planning/planner/public_road/open_space_planner.h"
#include "modules/planning/scenarios/park/valet_parking/auto_parking_scenario.h"


namespace apollo {

namespace planning {

using apollo::common::Status;
using apollo::common::TrajectoryPoint;

Status OpenSpacePlanner::Init(const PlanningConfig& config) {
    config_ = config;
    scenario_manager_.InitForAutoParking(config);
    return Status::OK();
}

Status OpenSpacePlanner::Plan(const TrajectoryPoint& planning_start_point,
                              Frame* frame,
                              ADCTrajectory* ptr_computed_trajectory) {
    // scenario_manager_.Update(planning_start_point, *frame);
    scenario_ = scenario_manager_.mutable_scenario();
    auto result = scenario_->Process(planning_start_point, frame);

    if (result == scenario::Scenario::STATUS_DONE) {
        // scenario_manager_.Update(planning_start_point, *frame);
    } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
        return Status(common::PLANNING_ERROR, "scenario returned unkown");
    }
    return Status::OK();
}


}  // namespace planning
}  // namespace apollo