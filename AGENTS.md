# Repository Guidelines

本文件用于定义 AI 与维护者在 OM 根仓内的行为规范与工作流边界。内容仅包含行为与流程，不重复技术细节。

其上级规范为C:\Users\Administrator\.codex\AGENTS.md，同样需要你参阅

## 1. 身份与边界
- 目标：优先保证技术正确性与可验证性。
- 不确定时必须明确说明，并请求进一步信息或验证。
- 不推测未证实内容，避免臆断。
- 任何重大变更（结构/工具链/构建入口）需先说明影响并获得确认。

## 2. 工作流（固定顺序）
1) 理解：读取必要上下文与文档索引。
2) 计划：复杂任务必须列 TODO 并逐步更新状态。
3) 执行：小步修改，保持一致性。
4) 验证：执行必要检查或测试。

## 3. 构建体系
- 和XMake有关

## 4. 文件与路径约束
- build 目录统一为 `build/<profile>`。
- 所有路径使用仓库相对路径表述。
- 不删除与任务无关的文件。

## 5. 文档一致性要求
- 构建体系相关文档以以下为准：
  - `oh-my-robot/docs/quick_start.md`
  - `oh-my-robot/docs/build/maintenance_manual.md`
- 构建流程变更必须同步更新上述两份文档。
- 新增构建文档时需在上述两份手册中建立索引或链接。

## 6. 变更记录与维护
- 重要流程变更需追加 `oh-my-robot/docs/build/maintenance_manual.md` 的“变更记录”小节。
- 历史/过程类文档仅作为归档参考，不作为现行标准。

## 7. 编码与格式
- 文档统一使用 UTF-8 编码。请使用UTF-8格式打开，UTF-8格式保存。
- 中文为主，术语保持一致（host/build profile）。

## 8. 并行开发（KISS）
- `main` 是 root 仓库唯一共享基线；并行任务统一从 `main` 切出，不长期保留专用同步 worktree。
- worktree 根目录固定为 `.worktrees/`，并保持忽略；完成任务后应及时删除对应 worktree。
- 一个任务只对应一套标识：一个 root 分支、一个 root worktree、一个同名的 `oh-my-robot` 子模块分支。
- root 分支命名遵循 `feature/<issue>-<slug>`、`fix/<issue>-<slug>`、`hotfix/<issue>-<slug>`；worktree 目录使用去斜杠形式，例如 `.worktrees/feature-123-osal-mutex/`。
- 创建任务 worktree 时，先从 `main` 切出 root 分支，再在 `oh-my-robot` 中从 `upstream/integration` 切出同名子模块分支；禁止为子模块再单独创建第二层 worktree。
- `oh-my-robot` 子模块远端拓扑固定为：`origin` 指向个人 Fork，`upstream` 指向官方仓库 `oh-my-robot/oh-my-robot-framework`。
- root 分支提交前应 rebase 到 `origin/main`；子模块分支提交前应 rebase 到 `upstream/integration`；统一禁止用 `merge` 同步上游。
- `robot/main` 只允许记录官方 `upstream/integration` 已可达的子模块提交，不得指向仅存在于个人 Fork 的临时提交。
