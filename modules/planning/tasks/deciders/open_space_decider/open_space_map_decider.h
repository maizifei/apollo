
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {

namespace planning {

class OpenSpaceMapDecider : public Decider {
    public:
    OpenSpaceMapDecider(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);

    private:
    apollo::common::Status Process(Frame* frame) override;

    private:
    apollo::common::VehicleParam vehicle_params_;

    common::VehicleState vehicle_state_;

    common::VehicleParam vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();

};

}  // namespace planning
}  // namespace apollo