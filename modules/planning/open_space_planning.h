
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/common/smoothers/smoother.h"
#include "modules/planning/planner/on_lane_planner_dispatcher.h"
#include "modules/planning/planner/public_road/open_space_planner.h"
#include "modules/planning/planning_base.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class OnLanePlanning
 *
 * @brief Planning module main class for open space scenario. It processes localization map and chassis as input,
 * to generate planning info.
 */

class OpenSpacePlanning : public PlanningBase {
    public:
    explicit OpenSpacePlanning::OpenSpacePlanning(const std::shared_ptr<DependencyInjector>& injector)
    : PlanningBase(injector) {

    }
    virtual ~OpenSpacePlanning();

    /**
     * @brief Planning name.
     * 
     */
    std::string Name() const override;

    /**
     * @brief module initilization function
     * @return initialization status
     */
    common::Status Init(const PlanningConfig& config) override;

    /**
     * @brief main logic of the planning module, runs periodically triggered by timmer.
     * 
     */
    void RunOnce(const LocalView& local_view,
                ADCTrajectory* const ptr_trajectory_pb) override;

    common::Status Plan(const double current_time_stamp,
                        const std::vector<common::TrajectoryPoint>& stitching_trajectory,
                        ADCTrajectory* const trajectory) override;

    bool CheckPlanningConfig(const PlanningConfig config);
    
    private:
    common::Status InitFrame(const uint32_t sequence_num,
                             const common::TrajectoryPoint& planning_start_point,
                             const common::VehicleState& vehicle_state);

    common::VehicleState AlignTimeStamp(const common::VehicleState& vehicle_state,
                                        const double curr_timestamp) const;

    private:
    std::unique_ptr<OpenSpacePlanner> open_space_planner_;

};



}
}