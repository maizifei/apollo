
#include "modules/planning/tasks/deciders/open_space_decider/open_space_map_decider.h"

#include <memory>
#include <utility>

#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {

namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::math::Polygon2d;
using apollo::common::PointENU;


OpenSpaceMapDecider::OpenSpaceMapDecider(const TaskConfig& config,
        const std::shared_ptr<DependencyInjector>& injector)
        : Decider(config, injector) {
    vehicle_params_ =
        apollo::common::VehicleConfigHelper::GetConfig().vehicle_param();
}

Status OpenSpaceMapDecider::Process(Frame* frame) {
    if (frame == nullptr) {
        const std::string msg = 
            "Invalid frame, fail to process the OpenSpaceRoiDecider.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    vehicle_state_ = frame->vehicle_state();


    return Status::OK();
}

}
}