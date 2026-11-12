# Heading integration decisions

Baseline: `815540bc640cc58975ab2d158c01ede960b99272` (`main`). Feature branch: `coal_scene_heading`.

| Local artifact | Repository destination | Decision |
|---|---|---|
| S01_subt_heading | src/coal_scene_tools/experiments/heading/scenes/S01_subt_heading | Add an explicitly scoped experiment profile |
| Kinematic trajectory | Existing coal_face_sim C++ plugin build | Add an experiment-only model plugin; execute the configured route on world updates without pausing the world |
| Runtime adapter | src/coal_scene_tools/experiments/heading/runtime | Adapt paths, startup synchronization, processing and diagnostics |
| Fixed-distance scripts and JSON | src/coal_scene_tools/experiments/lidar_fixed_distance_test | Preserve exact bytes; preserved_sha256.json verifies copies |
| SubT assets | Existing coal_face_sim asset catalog and restore_assets.py | Reuse; no new meshes/textures |
| Generic schema and loader | Existing coal_scene_tools | Unchanged; profile is not compatible with the mobile base_link schema |
| Public SimHeader/QueryRay/UWB modules | Existing packages | Unchanged; /heading topics are experiment-local, not a replacement public interface |
| Generated reports and logs | Ignored results/evidence directories | Keep locally; only compact reviewed acceptance evidence is tracked |

This source-run experiment is located outside the generic loader's scene discovery directory. It is not installed as a new ROS package and does not silently extend the public platform interface. The shared validator's duplicate-key/finite-value YAML parsing is reused. The generic geometry query engine is not duplicated: Gazebo supplies first-surface scan ranges; the added finite-segment zone integration serves a different concentration-input contract.

## Startup failure and repair

Historical failure: static109 frames/36Hz, baseline/mock about35.68Hz, two alignment rejects and a0.332s initial processing gap. Geometry/TF/Mock values passed. Existing logs cannot uniquely distinguish transport loss from startup scheduling, so CPU saturation is not asserted as the proven cause.

The previous adapter started timing at the first scan, used a depth5 best-effort subscription, drained pending frames inside one timer callback, and revalidated unchanged zones for every beam. The new adapter requires2s of stable scan/pose/clock readiness before formal sampling, uses bounded-depth256 reception, processes one frame per timer callback, and precompiles zone bounds once. Warmup gaps are reported separately. Formal raw reception, processing, alignment rejects, interval gaps and peak queue length are explicit. An independent subscriber verifies all formal frames for both static and motion, not just the first100 static frames.

Frequency remains 40 Hz in simulation time (measured mean tolerance ±1 Hz). Formal windows contain 120–121 frames for 3 s or 620–621 frames for 15.5 s: the one-frame range accounts for whether the final timestamp falls within the inclusive window. All received frames must be processed and independently paired across raw/baseline/Mock, with zero alignment rejects and no source interval reaching 50 ms. Intervals outside 24–26 ms remain reported as jitter, not silently discarded or automatically classified as lost frames. A 1 ns comparison epsilon handles floating-point boundaries. Gazebo runs with --lockstep and a configured real-time factor of 0.5 to leave processing headroom; the sensor remains 40 Hz in simulation time, not 40 Hz wall-clock time. All failed runs remain in local evidence, with no automatic retry masking failure. These gates detect missing processing and long source gaps; they do not prove perfectly uniform hardware timing or exhaustive stability.

## Native motion scheduling

The ROS `set_entity_state` upstream implementation pauses and resumes the world for each pose update ([source](https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/ros2/gazebo_ros/src/gazebo_ros_state.cpp)). Repeated 40 Hz calls are removed from this experiment. A small model plugin in the existing `coal_face_sim` package executes the validated route from the experiment start time during `WorldUpdateBegin`, without changing world pause state. The existing trajectory plugin and public interfaces remain unchanged. `std_msgs` is declared as a build/runtime dependency. The original 1 ms physics step is retained: a 5 ms trial worsened gaps and center error and was rejected. Actual P3D odometry remains the truth source; commands never substitute for measured pose. The first native pose-command trial still showed a 5 cm center error despite no long source gap, so high-frequency Python pose commands were also removed. The native trajectory receives one start timestamp, uses the configured speed and travel interval, and avoids delayed command jumps. The validator checks SDF trajectory parameters against demo_route.yaml. The final route version passed three consecutive headless full runs and one GUI full run; see HEADING_VALIDATION.md. This is evidence for the tested configuration, not proof that all scheduler failures are impossible.
