--- @file xmake.lua
--- @brief 项目构建入口
--- @details 定义顶层目标并挂载 OM 构建规则。
set_project("oh-my-robot")
set_xmakever("3.0.7")
add_rules("mode.debug", "mode.release")
set_policy("build.optimization.lto", false)
--- @brief 自动生成 compile_commands.json
--- @details 启用插件自动更新编译命令数据库。
add_rules("plugin.compile_commands.autoupdate", {outputdir = os.projectdir()})

includes("oh-my-robot")

--- @target robot_project
--- @brief 项目主可执行目标
--- @details 聚合 OM 静态库并挂载构建规则。
target("robot_project")
    set_kind("binary") -- 编译为可执行镜像
    set_filename("robot_project.elf")
    add_deps("tar_oh_my_robot")
    add_rules("oh_my_robot.context", "oh_my_robot.board_assets", "oh_my_robot.image_convert")
    add_files(path.join("oh-my-robot", "samples", "motor", "p1010b", "main.c"))
