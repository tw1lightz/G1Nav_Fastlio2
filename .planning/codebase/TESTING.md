# Testing Patterns

**Analysis Date:** 2026-05-06

## Test Framework

**No test framework is configured or used in this codebase.**

| Aspect | Status |
|--------|--------|
| Test runner | Not detected |
| Test configuration files | None (`jest.config.*`, `vitest.config.*`, `pytest.ini`, `setup.cfg`, etc.) |
| Test directories | None |
| Unit test files | None (`*_test.cpp`, `*_test.py`, `*.test.cpp`, `*.spec.cpp`) |
| CTest/ROS test targets | Not configured in `CMakeLists.txt` |

**Evidence:**

- `G1Nav2D/src/fastlio2/package.xml` — Contains `<!-- <test_depend>gtest</test_depend> -->` as a commented-out template line. No active test dependencies.
- `G1Nav2D/src/fastlio2/CMakeLists.txt` — No `catkin_add_gtest()`, `add_test()`, or any `ENABLE_TESTING` block.
- `G1Nav2D/src/CMakeLists.txt` — Only contains `add_subdirectory` directives, no test configuration.

## Test File Organization

**No test files exist.** The project has zero automated test coverage.

## Test Structure

**Not applicable** — no tests exist.

## Mocking

**Not applicable** — no mocking framework detected. The codebase does not use Google Mock, pytest-mock, or any other mocking library.

## Fixtures and Factories

**Not applicable** — no test fixtures or data factories exist.

PCD test data files exist at the repository root (`test1.pcd`, `test2.pcd`, `test3.pcd`, `test5.pcd`, `test6.pcd`, `test7.pcd`, `ground_test3.pcd`, `ground_test5.pcd`) but these are output artifacts from SLAM runs, not test fixtures.

## Coverage

**No coverage tooling configured.** No `gcov`, `lcov`, `Codecov`, or `Coveralls` integration present.

**No CI/CD pipeline detected.** The `.github/workflows/` directory is empty. No automated testing gates exist.

## Test Types

### Unit Tests
**Not used.** No unit tests for C++ classes (`LIOBuilder`, `IMUProcessor`, `IcpLocalizer`, `Pose6D`, etc.).

### Integration Tests
**Not used.** No integration tests for ROS node interactions or service callbacks.

### E2E / Manual Testing
**Manual testing only.** The project relies entirely on:
- Launch files for running hardware-in-the-loop tests
- RViz visualization for validating SLAM output
- ROS services (`slam_reloc`, `save_map`, `slam_hold`, `slam_start`) exposed for manual invocation

**ROS service-based interaction** is the closest thing to testability — services in `G1Nav2D/src/fastlio2/srv/` provide well-defined request/response interfaces:
| Service file | Purpose |
|---|---|
| `SaveMap.srv` | Request: save_path, resolution. Response: status, message |
| `SlamReLoc.srv` | Request: pcd_path, x, y, z, roll, pitch, yaw. Response: status, message |
| `SlamRelocCheck.srv` | Response: status (bool) |
| `SlamHold.srv` | Response: status, message |
| `SlamStart.srv` | Response: status, message |
| `MapConvert.srv` | Request: map_path, resolution, save_path. Response: status, message |

These SRV interfaces could be used for automated integration testing with `rosservice call`, but no such tests are implemented.

### Shell Script Tests

The `ros_map_edit` package (under `G1Nav2D/src/ros_map_edit/scripts/`) contains test shell scripts:

| Script | Purpose |
|---|---|
| `test_eraser.sh` | Manual test for map eraser tool |
| `test_file_dialog.sh` | Manual test for file dialog |
| `test_map_loading.sh` | Manual test for map loading |
| `test_map_loading_only.sh` | Minimal map loading test |
| `test_plugin.sh` | Manual test for plugin system |
| `test_save.sh` | Manual test for save functionality |
| `test_square_brush.sh` | Manual test for square brush tool |
| `test_with_new_map.sh` | Manual test with fresh map |

These are manual execution scripts (run by developer, no automated assertions).

## Common Patterns

**Not applicable** — no test patterns exist in the codebase.

## Recommendations

Given the complete absence of testing infrastructure, the following are the most impactful additions:

1. **Add Google Test dependency** — Uncomment `<test_depend>gtest</test_depend>` in `G1Nav2D/src/fastlio2/package.xml` and add `catkin_add_gtest()` targets in `CMakeLists.txt`.

2. **Unit test targets to prioritize:**
   - `IMUProcessor` (`include/lio_builder/imu_processor.h`, `src/lio_builder/imu_processor.cpp`)
   - `IcpLocalizer` (`include/localizer/icp_localizer.h`, `src/localizer/icp_localizer.cpp`)
   - Utility functions in `commons.h` / `commons.cpp` — `sq_dist()`, `rotate2rpy()`, `eigen2Odometry()`
   - `LoopClosureThread` methods — `getSubMaps()`, `loopCheck()`, `addOdomFactor()`

3. **Integration test targets to prioritize:**
   - Service callback logic — `saveMapCallback`, `relocCallback`, `mapConvertCallback` have well-defined request/response contracts suitable for `rostest`.
   - Data synchronization — `MeasureGroup::syncPackage()` has clear input/output behavior.

4. **CI pipeline:** Add `.github/workflows/` with at minimum:
   - Build verification on push
   - Test execution on pull request

---

*Testing analysis: 2026-05-06*
