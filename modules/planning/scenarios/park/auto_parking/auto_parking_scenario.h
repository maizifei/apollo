
#pragma once

#include <memory>
#include <string>

#include "modules/planning/scenarios/scenario.h"

namespace apollo {

namespace planning {

namespace scenario {

namespace auto_parking{

struct AutoParkingContext {
    ScenarioAutoParkingConfig scenario_config;
};


class AutoParkingScenario : public Scenario {
    public:
    AutoParkingScenario(const ScenarioConfig& config, const ScenarioContext* contex,
                        const std::shared_ptr<DependencyInjector>& injector)
        : Scenario(config, contex, injector) {}

    void Init() override;

    std::unique_ptr<Stage> CreateStage(const ScenarioConfig::StageConfig& stage_config,
                                       const std::shared_ptr<DependencyInjector>& injector) override;
    
    private:
    static void RegisterStages();
    bool GetScenarioConfig();

    private:
    static apollo::common::util::Factory<ScenarioConfig::StageType, Stage,
        Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
                   const std::shared_ptr<DependencyInjector>& injector)> s_stage_factory_;
    AutoParkingContext context_;
};


}  // namespace auto_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo