
#include "modules/planning/open_space_planning.h"

#include <algorithm>
#include <limits>
#include <list>
#include <utility>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/planning_semantic_map_config.pb.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {

namespace planning {

using apollo::canbus::Chassis;
using apollo::common::EngageAdvice;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::planning_internal::SLFrameDebug;
using apollo::planning_internal::SpeedPlan;
using apollo::planning_internal::STGraphDebug;
using apollo::common::math::Polygon2d;
using apollo::common::PointENU;

OpenSpacePlanning::~OpenSpacePlanning() {

    open_space_planner_->Stop();
    injector_->frame_history()->Clear();
    injector_->history()->Clear();
    injector_->planning_context()->mutable_planning_status()->Clear();
    injector_->ego_info()->Clear();
}

std::string OpenSpacePlanning::Name() const { return "open_space_planning"; }

Status OpenSpacePlanning::Init(const PlanningConfig& config) {
    config_ = config;

    if (!CheckPlanningConfig(config_)) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "planning config error: " + config_.DebugString());
    }

    PlanningBase::Init(config_);

    // planner_dispatcher_路径规划工厂类注册，包括rtk、publick_road、lattice等
    // planner_dispatcher_->Init();

    // 交通规则配置文件

    // clear planning history
    injector_->history()->Clear();

    // clear planning status
    injector_->planning_context()->mutable_planning_status()->Clear();

    // load map
    // reference line provider

    // 创建open_space_planner_对象
    open_space_planner_ = std::make_unique<OpenSpacePlanner>(injector_);

    return open_space_planner_->Init(config_);
}

Status OpenSpacePlanning::InitFrame(const uint32_t sequence_num,
                                    const TrajectoryPoint& planning_start_point,
                                    const VehicleState& vehicle_state) {
    frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                           vehicle_state));
    if (frame_ == nullptr) {
        return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
    }

    auto status = frame_->InitForOpenSpace(injector_->vehicle_state());

    if (!status.ok()) {
        AERROR << "failed to init frame:" << status.ToString();
        return status;
    }
    return Status::OK();
}

void OpenSpacePlanning::RunOnce(const LocalView& local_view,
                                ADCTrajectory* const ptr_trajectory_pb) {
    const double start_timestamp = Clock::NowInSeconds();
    const double start_system_timestamp = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();

    // localization
    ADEBUG << "Get localization:"
           << local_view_.localization_estimate->DebugString();

    // chassis
    ADEBUG << "Get chassis:" << local_view_.chassis->DebugString();

    Status status = injector_->vehicle_state()->Update(
        *local_view_.localization_estimate, *local_view_.chassis);

    VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();

    const double vehicle_state_timestamp = vehicle_state.timestamp();
    DCHECK_GE(start_timestamp, vehicle_state_timestamp)
        << "start_timestamp is behind vehicle_state_timestamp by"
        << start_timestamp - vehicle_state_timestamp << " secs";

    // if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    //     const std::string msg = "Update VehicleStateProvider failed "
    //                             "or the vehicle state is out dated.";
    //     AERROR << msg;
    //     ptr_trajectory_pb->mutable_decision()->mutable_main_decision()
    //                      ->mutable_not_ready()->set_reason(msg);
    //     status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    //     ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    //     FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    //     GenerateStopTrajectory(ptr_trajectory_pb);
    //     return;
    // }

    if (start_timestamp - vehicle_state_timestamp < FLAGS_message_latency_threshold) {
        vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
    }

    // 估计planning的循环时间
    const double planning_cycle_time = 
        1.0 / static_cast<double>(FLAGS_planning_loop_rate);

    std::string replan_reason;
    std::vector<TrajectoryPoint> stitching_trajectory = 
        TrajectoryStitcher::ComputeStitchingTrajectory(vehicle_state, start_timestamp,
            planning_cycle_time, FLAGS_trajectory_stitching_preserved_length, true,
            last_publishable_trajectory_.get(), &replan_reason);

    injector_->ego_info()->Update(stitching_trajectory.back(), vehicle_state);
    const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);

    // 更新Frame帧数据
    status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

    if (FLAGS_enable_record_debug) {
        frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
    }
    ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
        Clock::NowInSeconds() - start_timestamp);

    // 状态异常处理
    if (!status.ok()) {
        AERROR << status.ToString();
        if (FLAGS_publish_estop) {
            ADCTrajectory estop_trajectory;
            EStop* estop = estop_trajectory.mutable_estop();
            estop->set_is_estop(true);
            estop->set_reason(status.error_message());
            status.Save(estop_trajectory.mutable_header()->mutable_status());
            ptr_trajectory_pb->CopyFrom(estop_trajectory);
        }else {
            ptr_trajectory_pb->mutable_decision()->mutable_main_decision()
                ->mutable_not_ready()->set_reason(status.ToString());
            status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
            // GenerateStopTrajectory(ptr_trajectory_pb);
        }
        ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
        FillPlanningPb(start_timestamp, ptr_trajectory_pb);
        frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
        const uint32_t n = frame_->SequenceNum();
        injector_->frame_history()->Add(n, std::move(frame_));
        return;
    }

    // 进行运动规划
    status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);
    for (const auto& p : ptr_trajectory_pb->trajectory_point()) {
        ADEBUG << p.DebugString();
    }
    const auto end_system_timestamp = std::chrono::duration<double>(
                                        std::chrono::system_clock::now().time_since_epoch()).count();
    const auto time_diff_ms = (end_system_timestamp - start_system_timestamp) * 1000;
    ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

    ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
    ADEBUG << "Planning latency: " << ptr_trajectory_pb->latency_stats().DebugString();

    if (!status.ok()) {
        status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
        AERROR << "Planning failed:" << status.ToString();
        if (FLAGS_publish_estop) {
            AERROR << "Planning failed and set estop";
            EStop* estop = ptr_trajectory_pb->mutable_estop();
            estop->set_is_estop(true);
            estop->set_reason(status.error_message());
        }
    }

    ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
    if (ptr_trajectory_pb->is_replan()) {
        ptr_trajectory_pb->set_replan_reason(replan_reason);
    }

    if (frame_->open_space_info().is_on_open_space_trajectory()) {
        FillPlanningPb(start_timestamp, ptr_trajectory_pb);
        ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
        frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    }

    const uint32_t n = frame_->SequenceNum();
    injector_->frame_history()->Add(n, std::move(frame_));

}

common::VehicleState OpenSpacePlanning::AlignTimeStamp(const VehicleState& vehicle_state,
                                                       const double curr_timestamp) const {
    auto future_xy = injector_->vehicle_state()->EstimateFuturePosition(
        curr_timestamp - vehicle_state.timestamp());

    VehicleState aligned_vehicle_state = vehicle_state;
    aligned_vehicle_state.set_x(future_xy.x());
    aligned_vehicle_state.set_y(future_xy.y());
    aligned_vehicle_state.set_timestamp(curr_timestamp);
    return aligned_vehicle_state;
}

bool OpenSpacePlanning::CheckPlanningConfig(const PlanningConfig& config) {
    // TODO: add config params check 
    return true;
}

Status OpenSpacePlanning::Plan(const double current_time_stamp,
                               const std::vector<TrajectoryPoint>& stitching_trajectory,
                               ADCTrajectory* const ptr_trajectory_pb) {
    auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
    if (FLAGS_enable_record_debug) {
        ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
            stitching_trajectory.back());
        frame_->mutable_open_space_info()->set_debug(ptr_debug);
        frame_->mutable_open_space_info()->sync_debug_instance();
    }

    auto status = open_space_planner_->Plan(stitching_trajectory.back(), frame_.get(),
                                            ptr_trajectory_pb);

    ptr_debug->mutable_planning_data()->set_front_clear_distance(
        injector_->ego_info()->front_clear_distance());

    // enable start auto from open_space planner.
    if (frame_->open_space_info().is_on_open_space_trajectory()) {
        frame_->mutable_open_space_info()->sync_debug_instance();
        const auto& publishable_trajectory = 
            frame_->open_space_info().publishable_trajectory_data().first;
        const auto& publishable_trajectory_gear =
            frame_->open_space_info().publishable_trajectory_data().second;
        publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
        ptr_trajectory_pb->set_gear(publishable_trajectory_gear);

        auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

        if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
            Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
            engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
            engage_advice->set_reason("Ready to engage when starting with OPEN_SPACE_PLANNER");
        } else {
            engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
            engage_advice->set_reason("Keep engage while in parking");
        }

        ptr_trajectory_pb->mutable_decision()->mutable_main_decision()
            ->mutable_parking()->set_status(MainParking::IN_PARKING);

    }

    return status;
}

}
}