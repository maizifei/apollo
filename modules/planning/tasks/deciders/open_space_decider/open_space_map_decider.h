
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
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/indexed_queue.h"
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

    // @brief generate the path by vehicle location and return the target parking
    // spot on that path
    bool GetParkingSpot(Frame *const frame,
                        std::array<common::math::Vec2d, 4> *vertices);

    // @brief Set an origin to normalize the problem for later computation
    void SetOrigin(Frame *const frame,
                    const std::array<common::math::Vec2d, 4> &vertices);

    void SetParkingSpotEndPose(
        Frame *const frame, const std::array<common::math::Vec2d, 4> &vertices);

    // @brief "Region of Interest", load map boundary for open space scenario
    // @param vertices is an array consisting four points describing the
    // boundary of spot in box. Four points are in sequence of left_top,
    // left_down, right_down, right_top
    // ------------------------------------------------------------------
    //
    //                     --> lane_direction
    //
    // ----------------left_top        right_top--------------------------
    //                -                  -
    //                -                  -
    //                -                  -
    //                -                  -
    //                left_down-------right_down
    bool GetParkingBoundary(Frame *const frame,
                            const std::array<common::math::Vec2d, 4> &vertices,
                            std::vector<std::vector<common::math::Vec2d>>
                                *const roi_parking_boundary);

    // @brief Helper function for fuse line segments into convex vertices set
    bool FuseLineSegments(
        std::vector<std::vector<common::math::Vec2d>> *line_segments_vec);

    // @brief main process to compute and load info needed by open space planner
    bool FormulateBoundaryConstraints(
        const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
        Frame *const frame);

    // @brief Represent the obstacles in vertices and load it into
    // obstacles_vertices_vec_ in clock wise order. Take different approach
    // towards warm start and distance approach
    bool LoadObstacleInVertices(
        const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
        Frame *const frame);

    bool FilterOutObstacle(const Frame &frame, const Obstacle &obstacle);

    // @brief Transform the vertice presentation of the obstacles into linear
    // inequality as Ax>b
    bool LoadObstacleInHyperPlanes(Frame *const frame);

    // @brief Helper function for LoadObstacleInHyperPlanes()
    bool GetHyperPlanes(const size_t &obstacles_num,
                        const Eigen::MatrixXi &obstacles_edges_num,
                        const std::vector<std::vector<common::math::Vec2d>>
                            &obstacles_vertices_vec,
                        Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);

    private:
    apollo::common::VehicleParam vehicle_params_;

    common::VehicleState vehicle_state_;

    common::VehicleParam vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();

    ThreadSafeIndexedObstacles *obstacles_by_frame_;

};

}  // namespace planning
}  // namespace apollo