-- Example preset for local configuration.
-- Update these values to match your environment.
--- @param preset table 预设配置表
--- @field board table 板级配置
--- @field os table 操作系统配置
--- @field toolchain_default table 默认工具链配置
--- @field toolchain_presets table 工具链预设配置
--- @field flash table 烧录预设配置
local preset = {
  board = {name = "rm-c-board"},
  os = {name = "freertos"},
  toolchain_default = {
    name = "armclang",
  },
  toolchain_presets = {
    ["armclang"] = {
      sdk = "D:/Program Files/ProgramTools/Keil_v5/ARM/ARMCLANG",
      bin = "D:/Program Files/ProgramTools/Keil_v5/ARM/ARMCLANG/bin",
    },
    ["gnu-rm"] = {
      sdk = "D:/Program Files/ProgramTools/gcc-arm-none-eabi-10.3-2021.10",
      bin = "D:/Program Files/ProgramTools/gcc-arm-none-eabi-10.3-2021.10/bin",
    },
  },
  flash = {
    jlink = {
      device = "STM32F407IG",
      interface = "swd",
      speed = 4000,
      program = "D:/Program Files/ProgramTools/SEGGER/Jlink/JLink.exe",
      target = "robot_project",
      firmware = nil,
      prefer_hex = true,
      reset = true,
      run = true,
    },
  },
}

function get_preset()
  return preset
end
