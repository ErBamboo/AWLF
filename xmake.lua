--- @file xmake.lua
--- @brief 项目构建入口
--- @details 定义顶层目标并挂载 AWLF 构建规则。
set_project("awlf")
add_rules("mode.debug", "mode.release")
--- 自动生成 compile_commands.json
add_rules("plugin.compile_commands.autoupdate", {outputdir = os.projectdir()})

includes("awlf")

--- @target robot_project
--- @brief 项目主可执行目标
--- @details 聚合 AWLF 静态库并挂载构建规则。
target("robot_project")
    set_kind("binary") -- 编译为可执行镜像
    set_filename("robot_project.elf")
    add_deps("tar_awlf")
    set_policy("check.auto_ignore_flags", false)
    add_rules("awlf.context", "awlf.board_assets", "awlf.image_convert")
    add_files("awlf/samples/stm32f407/osal_sync/main.c")
