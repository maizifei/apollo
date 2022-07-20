
#pragma once

#include <memory>

#include "modules/planning/scenarios/park/auto_parking/auto_parking_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {

namespace planning {

namespace scenario {

namespace auto_parking{

class StageAutoParking : public Stage {
    public:
    StageAutoParking(const ScenarioConfig::StageConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector)
        : Stage(config, injector) {}

    private:
    Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

    AutoParkingContext* GetContext() {
        return GetContextAs<AutoParkingContext>();
    }

    private:
    Stage::StageStatus FinishStage();

    private:
    ScenarioAutoParkingConfig scenario_config_;
};

}  // namespace auto_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo