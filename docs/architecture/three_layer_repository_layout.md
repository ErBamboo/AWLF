# 三层仓库拓扑说明

本文档用于固定当前工作区的三层职责边界，避免后续再次把底座框架、领域模块和项目应用混在一起。

## 仓库职责
- `robot/`
  - 当前根仓，承担具体项目应用层。
  - 负责 `main`、项目构建入口、参数表、设备 ID、标定值、任务装配和最终镜像产出。
- `omr-robotics/`
  - `robot/` 内的领域应用库子模块。
  - 对外暴露领域模块，并在其内部再引入 `oh-my-robot/` 底座子模块。
- `omr-robotics/oh-my-robot/`
  - 由 `omr-robotics` 递归带入的底座子模块。
  - 对项目层透明，不再作为 `robot/` 的直接依赖入口。
- `../worktrees/oh-my-robot/<task>/`
  - `oh-my-robot` 的日常开发工作区。
  - 用于框架层文档、构建系统、OSAL、drivers、BSP 等底座能力演进。

## 当前约束
- `robot/` 不再直接使用 `omr-robotics/oh-my-robot/samples/...` 作为长期项目入口。
- `robot/` 只直接依赖 `omr-robotics/`，不再直接依赖 `oh-my-robot/`。
- 新增机器人业务模块时，优先落到 `omr-robotics/`，而不是继续新增到底座仓。
- `oh-my-robot` 不再承载正式业务子系统层；`lib/systems/` 已从框架仓移除。

## 近期迁移顺序
1. 先让 `robot/` 拥有自己的 `app/main.c`。
2. 将 `omr-robotics/` 作为独立仓库引入 `robot/`。
3. 由 `omr-robotics/` 内部递归带入 `oh-my-robot/` 底座子模块。
4. 后续继续把具体业务实现补齐到 `omr-robotics/`。
