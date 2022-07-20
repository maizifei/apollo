
#include "modules/planning/scenarios/park/auto_parking/auto_parking_scenario.h"
#include "modules/planning/scenarios/park/auto_parking/stage_auto_parking.h"


namespace apollo {

namespace planning {

namespace scenario {

namespace auto_parking{

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

apollo::common::util::Factory<ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config, const std::shared_ptr<DependencyInjector>& injector)>
    AutoParkingScenario::s_stage_factory_;

void AutoParkingScenario::Init() {
    Scenario::Init();

    if (!GetScenarioConfig()) {
        AERROR << "fail to get scenario specific config";
        return;
    }
}

void AutoParkingScenario::RegisterStages() {
    if (s_stage_factory_.Empty()) {
        s_stage_factory_.Clear();
    }
    s_stage_factory_.Register(ScenarioConfig::AUTO_PARKING_PARKING,
        [](const ScenarioConfig::StageConfig& config,
           const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
           return new StageAutoParking(config, injector);
           });
}

std::unique_ptr<Stage> AutoParkingScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
    if (s_stage_factory_.Empty()) {
        RegisterStages();
    }
    auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(), stage_config, injector);

    if (ptr) {
        ptr->SetContext(&context_);
    }
    return ptr;
}

bool AutoParkingScenario::GetScenarioConfig() {
    if (!config_.has_auto_parking_config()) {
        AERROR << "miss scenario specific config";
        return false;
    }
    context_.scenario_config.CopyFrom(config_.auto_parking_config());
    return true;
}


}  // namespace auto_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo