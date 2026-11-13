# Third-party provenance

SubT source reference: https://github.com/osrf/subt , branch `fortress-noetic`, commit `ba15f9ff35ccfb43f44937d6d84e047615d174ff`.

This project reuses environment resources; it does not run or redistribute the SubT ROS 1 control stack.

The ROS 2 scene assembly and trajectory plugin target Gazebo Classic.

Pinned OpenRobotics Fuel resources:

| Local ID                 | Fuel model          | Version |
| ------------------------ | ------------------- | ------- |
| subt_tunnel_tile_1       | Tunnel Tile 1       | 5       |
| subt_tunnel_tile_2       | Tunnel Tile 2       | 16      |
| subt_tunnel_tile_5       | Tunnel Tile 5       | 14      |
| subt_tunnel_tile_blocker | Tunnel Tile Blocker | 12      |

`asset_manifest.json` retains model `authorship/source` identifiers and the hashes of the 117 archived files. `asset_sources.json` records pinned file URLs, donor texture origins and the COLLADA image-path replacements required to reproduce the localized archive.

Mesh geometry is preserved; paths and `material` names are localized, PBR blocks unsupported by Classic are removed, shared textures are resolved, and competition-only plugins are omitted.

This workspace retains the mesh and texture files required for offline simulation alongside the source. GitHub backs up this same workspace layout.

Model authorship remains in `model.config`.

Third-party resources remain subject to their original terms; this repository does not apply a blanket license to third-party resources.

No project-wide open-source license has been selected for the original platform code yet.

## 已核查的模型许可（2026-11-13）

公开 Fuel 模型 API 的 license_name/license_url 返回如下：

| 模型 | 许可 |
| --- | --- |
| Tunnel Tile 1 | CC BY 4.0 — https://creativecommons.org/licenses/by/4.0/ |
| Tunnel Tile 2 | CC0 1.0 — https://creativecommons.org/publicdomain/zero/1.0/ |
| Tunnel Tile 5 | CC0 1.0 — https://creativecommons.org/publicdomain/zero/1.0/ |
| Tunnel Tile Blocker | CC0 1.0 — https://creativecommons.org/publicdomain/zero/1.0/ |

模型来源：https://fuel.gazebosim.org/1.0/OpenRobotics/models/ （模型名按URL编码追加）。模型版本见本文上表，文件来源和SHA256见 support 中的 asset_sources.json 与 asset_manifest.json。
保留各模型 model.config/metadata.pbtxt 中的作者信息。当前副本已本地化纹理路径、材质及 Classic 兼容配置；这些修改不是上游原始版本。借用 Tile 1 的纹理继续按 CC BY 4.0 标注来源。

工作区 upstream/gazebo_cave_world 使用其自带 MIT LICENSE，保留原版权声明。
