# SubT heading scene

Four original-size Tunnel Tile 5 segments form a nominal80m corridor with a blocker. Assets are shared with `coal_face_sim/scenarios/coal_face_v1/models`; see the repository THIRD_PARTY.md and pinned asset manifests. No duplicate meshes are shipped here.

The fixed station and body-mounted target use the map frame tree. This `heading-runtime-1` profile is intentionally separate from the legacy `0.1.0` mobile-sensor scene schema: do not pass it to the generic scene_loader. Use the heading experiment runner. Legacy schemas/loaders remain unchanged.

EBZ200H overall envelope X/Y/Z=3.2/11.6/2.43m. Official body height is2.0m;2.43m is the highest point with the cutting head horizontal. A simple envelope does not reproduce the machine shape.

Respirable dust2/10/20mg/m³ is an uncalibrated test scenario. The high face zone is mathematically tested but not sampled by live rays in this occluded layout. Motion is kinematic, not track/contact dynamics. Mock is a constant+0.02m and ignores concentration. See docs/HEADING_QUICKSTART.md at repository root for commands and limitations.
