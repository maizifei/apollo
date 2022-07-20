load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "auto_parking",
    srcs = [
        "stage_auto_parking.cc",
        "auto_parking_scenario.cc",
    ],
    hdrs = [
        "stage_auto_parking.h",
        "auto_parking_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/scenarios:scenario",
    ],
)

cpplint()
