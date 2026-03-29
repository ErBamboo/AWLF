# 三层仓库拓扑说明

本文档用于固定当前工作区的三层职责边界，避免后续再次把底座框架、领域模块和项目应用混在一起。

## 仓库职责
- `robot/`
  - 当前根仓，承担具体项目应用层。
  - 负责 `main`、项目构建入口、参数表、设备 ID、标定值、任务装配和最终镜像产出。
- `../omr-robotics/`
  - 领域应用库仓库骨架。
  - 负责机器人业务模块，例如 `chassis`、`gimbal`、`arm`、`referee`、`supercap`、整机状态机与机构编排。
- `oh-my-robot/`
  - `robot/` 内的集成快照。
  - 只用于项目聚合、联调验证、子模块指针收敛与发布前检查。
- `../worktrees/oh-my-robot/<task>/`
  - `oh-my-robot` 的日常开发工作区。
  - 用于框架层文档、构建系统、OSAL、drivers、BSP 等底座能力演进。

## 当前约束
- `robot/` 不再直接使用 `oh-my-robot/samples/...` 作为长期项目入口。
- `oh-my-robot` 不再承载正式业务子系统层；`lib/systems/` 已从框架仓移除。
- 新增机器人业务模块时，优先落到 `../omr-robotics/`，而不是继续新增到 `oh-my-robot/`。
- `robot/` 现在在顶层显式组合 `oh-my-robot` 与 `../omr-robotics/`，不再把领域层隐含塞进底座聚合目标。

## 近期迁移顺序
1. 先让 `robot/` 拥有自己的 `app/main.c`。
2. 再把 `oh-my-robot/lib/systems/` 中的过渡内容迁移到 `../omr-robotics/`。
3. 让 `robot/` 在依赖管理层显式组合 `oh-my-robot` 与 `omr-robotics`。
4. 后续继续把具体业务实现从 `oh-my-robot/lib/systems/` 分批迁出。
