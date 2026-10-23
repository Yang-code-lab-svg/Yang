# 反作用飞轮速度控制仿真：从标准 C 入门

这个项目面向 C 语言初学者，用普通电脑学习反作用飞轮的采样、滤波、速度 PI 和输出限幅。全部源码使用标准 C，不需要开发板、MATLAB、专有固件或实际在轨数据。

已实现 **20 Hz 控制、连续 5 拍转速均值、速度 PI、±0.04 N·m 力矩限幅、积分抗饱和，以及简化飞轮模型**。所有参数、指令、噪声和结果均为教学合成数据。

## 先运行起来：Windows

### 已有 C 编译器

1. 在本仓库选择 **Code → Download ZIP** 并解压；或用 Git 克隆：

   ```bat
   git clone https://github.com/Yang-code-lab-svg/Yang.git
   cd Yang
   ```

2. 打开命令提示符，进入包含 `README.md` 的项目根目录。
3. 执行：

   ```bat
   build_windows.bat
   ```

脚本优先使用 PATH 中的 GCC，其次使用 MSVC。它会依次编译仿真程序、编译测试、执行测试、运行仿真；任一步失败都会返回非零退出码。

成功时可看到 `42 checks, 0 failures`，并生成：

| 文件 | 用途 |
| --- | --- |
| `build\wheel_sim.exe` | 仿真程序 |
| `build\wheel_tests.exe` | 基础测试程序 |
| `build\simulation.csv` | 2400 拍完整仿真数据 |

可单独重跑：

```bat
build\wheel_tests.exe
build\wheel_sim.exe --output build\simulation.csv
build\wheel_sim.exe --no-noise --output build\no_noise.csv
build\wheel_sim.exe --help
```

同名 CSV 会被覆盖。输出文件夹需要事先存在。控制台使用英文，避免不同 Windows 代码页导致显示乱码；源码注释和本文说明使用中文。

### 还没有编译器：选择一种方式即可

**方式 A：MinGW-w64 GCC（通过 MSYS2 安装）。** 按 [MSYS2 官方安装说明](https://www.msys2.org/)安装，然后在 **MSYS2 UCRT64** 终端执行：

```sh
pacman -S --needed mingw-w64-ucrt-x86_64-gcc
```

之后在普通 Windows 命令提示符中，为本次会话添加编译器路径，再进入项目执行：

```bat
set "PATH=C:\msys64\ucrt64\bin;%PATH%"
gcc --version
build_windows.bat gcc
```

如果 MSYS2 安装位置不同，相应修改路径。采用 UCRT64 环境的依据见 [MSYS2 环境说明](https://www.msys2.org/docs/environments/)。

**方式 B：Visual Studio 的 C/C++ 工具。** 安装 Visual Studio 或 Build Tools 的“使用 C++ 的桌面开发”组件及 Windows SDK。在 **x64 Native Tools Command Prompt** 中进入项目，执行：

```bat
cl
build_windows.bat msvc
```

本项目在 MSVC 下使用 `/std:c11` 编译同一份源码，要求支持该选项的工具链。参见 [Microsoft 官方 C11/C17 支持说明](https://learn.microsoft.com/en-us/cpp/overview/install-c17-support?view=msvc-170)。

### 可选：CMake 构建

如果已安装 **CMake 3.21 或更高版本**及 C 编译器，可以使用：

```bat
cmake -S . -B build-cmake
cmake --build build-cmake --config Release
ctest --test-dir build-cmake -C Release --output-on-failure
```

Visual Studio 生成器的程序通常位于 `build-cmake\Release\`；GCC 单配置生成器的程序通常位于 `build-cmake\`。CTest 在构建目录运行仿真，CSV 位于 `build-cmake\simulation.csv`。

### Linux / macOS 手动构建示例

在已安装 GCC 或兼容 C 编译器的终端中执行；macOS 可把 `gcc` 替换为 `cc`：

```sh
mkdir -p build
gcc -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -Iinclude src/moving_average.c src/pi_controller.c src/wheel_model.c src/simulation.c src/main.c -o build/wheel_sim -lm
gcc -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -Iinclude src/moving_average.c src/pi_controller.c src/wheel_model.c src/simulation.c tests/test_main.c -o build/wheel_tests -lm
./build/wheel_tests
./build/wheel_sim --output build/simulation.csv
```

## 20 Hz 在这里如何实现

控制周期 `Ts = 1 / 20 = 0.05 s`。`simulation_step()` 每调用一次，就执行一次“采样 → 5 拍均值 → PI → 限幅 → 输出保持”，并推进 **50 ms 仿真时间**。

用整数 `tick` 安排指令和摩擦变化事件，记录时间为 `tick × Ts`。120 s 共执行 2400 次控制，事件不会因反复累加浮点时间而提前或延后。模型在两次控制之间用 50 个 1 ms 子步积分；这不会把控制频率变成 1000 Hz。

**这是可重复的仿真定时。** 程序通常很快跑完 120 s 的仿真，不会在真实电脑上等待 120 s。标准 C 没有跨平台的硬实时定时保证。本版未实现 Windows 墙钟实时调度；迁移开发板时，应由硬件定时器每 50 ms 触发一次控制任务。

## 运行场景与预期现象

| 仿真时间 | 合成输入 | 应观察的行为 |
| --- | --- | --- |
| 0 ≤ t < 2 s | 给定 0 rpm | 静止，测量含小幅确定性噪声 |
| 2 ≤ t < 30 s | 给定 1000 rpm | 启动力矩先限幅，随后速度接近给定 |
| 30 ≤ t < 60 s | 保持 1000 rpm；干摩擦幅值由 0.002 增至 0.008 N·m | 转速暂降，积分输出增加，速度恢复 |
| 60 ≤ t < 95 s | 给定改为 −500 rpm | 先制动再反向加速，最终接近 −500 rpm |
| 95 ≤ t < 120 s | 给定改为 0 rpm | 转子停止 |

默认带噪声运行的参考结果：29 s 转速约 1000.526 rpm，摩擦变化后的最低转速约 990.827 rpm，59 s 恢复至约 999.988 rpm；稳态电机力矩比摩擦变化前增加约 0.0060 N·m。94 s 约为 −499.708 rpm，最后 5 s 转速绝对值小于 1 rpm。

参考值用于理解响应；自动测试采用容差判定，允许不同编译器末位浮点结果略有差异。详见 [测试与验证记录](docs/validation.md)、[默认运行输出](examples/expected_console.txt)和[基础测试输出](examples/expected_tests.txt)。

## 初学者应怎样读源码

| 阅读顺序 | 文件 | 重点 |
| --- | --- | --- |
| 1 | `include/config.h` | 宏、控制周期、统一参数与单位 |
| 2 | `src/moving_average.c` | 数组、结构体、指针、循环缓冲区 |
| 3 | `src/pi_controller.c` | 误差、积分状态、条件判断、限幅 |
| 4 | `src/wheel_model.c` | rpm 与 rad/s 换算、力矩平衡、数值积分 |
| 5 | `src/simulation.c` | 20 Hz 任务顺序、合成指令、测量噪声 |
| 6 | `src/main.c` | 程序入口、命令行、CSV 文件、错误退出 |
| 7 | `tests/test_main.c` | 用独立预期值和闭环结果验证行为 |

`include/` 中的 `.h` 文件描述结构体和函数接口，`src/` 中的 `.c` 文件实现功能。项目不使用动态内存、线程、芯片寄存器或第三方 C 库。

### 5 拍转速均值

窗口满后：`filtered[k] = (measured[k] + ... + measured[k-4]) / 5`。每拍保留最新值并移除最旧值，不是每 5 拍才输出一次。开始的第 1～4 拍仅对已有样本求均值。

在 20 Hz 下，5 个采样时刻覆盖 0.20 s；稳定窗口对低频变化产生约 0.10 s 的群时延。平滑能减小测量波动，但会增加反馈延迟，因此 PI 参数按含滤波的完整闭环验证。

### 速度 PI、限幅与抗饱和

```text
error_rpm = target_rpm - filtered_rpm
P_nm = Kp * error_rpm
candidate_I_nm = I_nm + Ki * error_rpm * Ts
motor_torque_nm = clamp(P_nm + I_nm, -0.04, +0.04)
```

如果候选输出已越过某侧限值，且误差继续把输出推向同侧，保留原积分；否则更新积分，并约束积分值。这样可以避免长时间限幅时积分无限累积。`pi_update()` 的返回值直接作为本教学模型的电机力矩输入。

### 飞轮模型和符号

模型采用 `J * dω/dt = T_motor - T_friction`。转动时 `T_friction = b * ω + Tc * sign(ω)`；静止且 `|T_motor| ≤ Tc` 时，静摩擦平衡驱动力矩。这里把静摩擦阈值简化为同一个 `Tc`。

正电机力矩使飞轮角速度向正方向变化。`friction_torque_nm` 按实际转动方向带符号，在方程中减去。输出是**作用于飞轮转子的电机力矩**，并不是卫星本体净输出力矩；未建立卫星姿态或电流、电压、PWM 模型。

| 参数 | 合成设定 | 单位 |
| --- | --- | --- |
| 控制频率 | 20 | Hz |
| 转速窗口 | 5 | 拍 |
| 比例增益 Kp | 0.0005 | N·m/rpm |
| 积分增益 Ki | 0.0001 | N·m/(rpm·s) |
| 电机力矩范围 | −0.04 ～ +0.04 | N·m |
| 转动惯量 J | 0.005 | kg·m² |
| 黏性摩擦系数 b | 0.00002 | N·m/(rad/s) |
| 干摩擦幅值 Tc | 0.002，30 s 后为 0.008 | N·m |

这些数值是为教学响应选择的合成参数，不是型号标定数据。离散 PI、简化摩擦和显式 Euler 模型适用于本例；改变增益、惯量或积分步长后应重新验证稳定性。本项目不直接驱动真实飞轮。

## CSV 怎样核对

| 列 | 含义 |
| --- | --- |
| `time_s` | 本控制区间起点，0.000～119.950 s，间隔 0.050 s |
| `target_rpm` | 转速给定 |
| `true_rpm` | 模型真实转速；在本拍施加新输出之前记录 |
| `measured_rpm` | 真实转速叠加合成测量噪声 |
| `filtered_rpm` | 最近最多 5 拍测量值的均值 |
| `motor_torque_nm` | PI 限幅后的电机力矩，本拍保持 50 ms |
| `friction_torque_nm` | 本拍起点的有符号摩擦力矩 |
| `integral_nm` | PI 积分项 |
| `saturated` | 本拍最终输出到达限值时为 1，否则为 0 |

CSV 含 1 行表头和 2400 行数据；最后一行的输出作用到 120.000 s，控制台最终速度对应这个终点。

用 Excel 打开 CSV，插入**散点图并连线**：横轴选 `time_s`，纵轴分别选 `target_rpm`、`true_rpm`、`filtered_rpm`。再单独画 `motor_torque_nm` 和 `friction_torque_nm`，观察 30 s 后电机力矩如何提高。

## 测试与学习练习

基础测试包含 42 个检查，覆盖采样边界、滤波启动与环形覆盖、积分周期因子、双向限幅与抗饱和、物理单位、静摩擦、无驱动停转，以及有/无噪声的完整闭环场景。CTest 还检查 CSV 程序能正常结束及错误命令行返回失败。测试不用会被 Release 模式关闭的 `assert`。

GitHub Actions 配置了 Linux GCC、Windows MSVC 的 CMake 测试，以及 Windows 批处理构建测试。实际执行状态见仓库 [Actions 页面](https://github.com/Yang-code-lab-svg/Yang/actions)；通过的任务会提供 CSV 下载。

建议每次只改一个点，重跑测试并观察 CSV：

1. 把 5 拍窗口改为 1 拍，对比噪声与响应延迟；先恢复配置再做下一题。
2. 把 Ki 暂时改成 0，观察摩擦增大后纯比例控制的稳态转速误差。
3. 减小力矩上限，观察加速时间和限幅持续拍数。
4. 在理解现有函数后，增加 `STOP / RUN / FAULT` 状态机及故障复位测试。

修改参数后，原来的预期结果不一定成立；应先解释变化原因，再调整有依据的测试容差。后续可逐步加入定时器接口、传感器异常、超速保护和故障锁存。
