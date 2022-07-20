
#pragma once

#include <memory>
#include <string>

#include "modules/common/status/status.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 * 
 */

namespace apollo {

namespace planning{


/**
 * @class OpenSpacePlanner
 * @brief OpenSpacePlanner is a planner excuted on open space
 */

class OpenSpacePlanner : public Planner {
    public:

    OpenSpacePlanner() = delete;

    explicit OpenSpacePlanner(const std::shared_ptr<DependencyInjector>& injector) 
        :Planner(injector){}

    virtual ~OpenSpacePlanner() = default;

    void Stop() override {}

    std::string Name() override { return "OPEN_SPACE"; }

    common::Status Init(const PlanningConfig& config) override;

    common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                        Frame* frame,
                        ADCTrajectory* ptr_computed_trajectory) override;
};


}
}