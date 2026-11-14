# 删除本地环境后的恢复说明

## 1. 已备份到 GitHub 的范围

策略、工具、继承 Python 代码和测试夹具直接在 Git 中。A/C 二进制、原版回退文件、实际部署 SDK/配置及离线测试 wheels 放在本仓库 Release。下载脚本使用仓库中的固定 SHA-256 清单，下载完即核验。

`python3 recovery/verify.py` 检查原候选 48 的全部 230 个受控文件；`--runtime` 进一步检查附件中的实际运行文件。无需原机器的 `/home/krist`、`/media/krist/Data` 或旧容器来取回这些内容。旧报告里保留的绝对路径是来源记录，不是离线测试依赖。

不备份 Docker 镜像（用户明确要求）、操作系统、GPU 驱动和全部历史录像。删除后要运行仿真，需要重新获取官方镜像和系统依赖；官方 ROS 工作区源码已作为 Git 归档保存在本仓库 Release，编译产物需要按下文重建。未进行删除整机环境后的端到端抓取重装测试，不能把代码恢复验证表述为所有硬件、所有外部下载源永久可用。

## 2. 取回代码和附件

```bash
gh auth login
gh auth setup-git
gh repo clone Krist21a/scene2-visual
cd scene2-visual
git checkout scene2-v48-20261115
python3 recovery/verify.py
python3 recovery/download_assets.py --runtime --wheels
```

私有仓库需要原 GitHub 账号或被授予访问权限的账号。附件恢复到仓库中的 `runtime/`、`vendor/scene1/controllers/`、`downloads/wheels/`；这些目录不重复提交到 Git。

需要本轮原始证据时运行 `python3 recovery/download_assets.py --evidence`。原始 summary 的自动结果为 5/6，人工确认 6/6 见 `docs/validation/user-confirmation-v48.json`。

## 3. 无 ROS 的离线恢复验证

推荐 CPython 3.12、Linux x86_64，并具备 `venv` / `pip`。Ubuntu 可安装相应 Python 版本的 venv 包。

```bash
PYTHON_BIN=python3.12 bash recovery/setup_offline.sh
bash recovery/run_tests.sh
```

附带 wheels 仅对应 CPython 3.12 / Linux x86_64；Python 3.10–3.12 的其他兼容系统可不下载 wheels，直接联网安装 `requirements-offline.txt`。其他 Python 版本或架构未验证。

原生产环境是 Python 3.12.12、NumPy 1.26.4、SciPy 1.15.3、OpenCV headless 4.13.0.92、PyYAML 6.0.3。其中 OpenCV 4.13 声明要求 NumPy>=2，而原环境实际使用 NumPy 1.26；这是原有依赖元数据不一致，已明确保留记录。离线恢复采用支持 NumPy 1.26 的 OpenCV 4.11.0.86，具体验证结果见 `docs/validation/clean-recovery-result.json`。离线通过不等于新的多种子物理验收通过。

## 4. Docker 镜像和官方工作区（未上传）

本机使用的镜像与容器名称：

| 用途 | 镜像 | 原容器名 |
| --- | --- | --- |
| 官方仿真 | `kuavo_challenge_cup_2026:latest` | `kuavo_challenge_container_GPU_046e4301` |
| 策略 | 本机自建 `kuavo-ros-noetic:s1-s2-tested` | `kuavo-ros-noetic-s12-ready` |

`tools/workstation.py` 使用上述固定容器名。恢复时按这个名字创建，或自行修改工具并重新验证。

官方工作区来源：<https://gitcode.com/OpenLET/leju-kuavo-challenge-cup-2026.git>

本次工作区提交：`51b3defaf8c032957647c7aa193d1fa20daef1f3`。优先从本仓库附件恢复固定源码：

```bash
python3 recovery/download_assets.py --simulator
git clone downloads/scene2-simulator-51b3def.bundle simulator-workspace
cd simulator-workspace
git checkout 51b3defaf8c032957647c7aa193d1fa20daef1f3
```

该 Git bundle 自包含全部所需历史/对象，不含原工作区的 `.git/config`、凭据、构建日志或 Docker 层。源码内第三方 DDS 测试附带的公开测试密钥属于其上游测试夹具，不是用户凭据。

原工作区 Git 源码未修改；控制器二进制部署在 `devel/` 中，必须单独恢复本仓库提供的 A/C 文件。

官方镜像下载地址取自该提交的 `readme.md`：

```bash
wget https://kuavo.lejurobot.com/challenge_cup_2026/kuavo_challenge_cup_2026_latest.tar.gz
docker load -i kuavo_challenge_cup_2026_latest.tar.gz
# 进入上一步从 Git bundle 恢复的 simulator-workspace。
./docker/run_with_gpu_for_challenge.sh
```

`latest` 地址内容可能更新。原镜像 ID 为 `sha256:8eafa6a83f9edeb93e28da7cabe6b930fcccd88f02b4c3b57668549b6bae7e50`，应与下载后的镜像比较；不一致时需要重新验证兼容性。镜像获取及版权遵循赛事提供方要求。

容器内按官方说明编译：

```bash
cd /root/kuavo_ws
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
source installed/setup.zsh
catkin build kuavo_msgs challenge_cup_simulator challenge_cup_task_template humanoid_controllers
source devel/setup.zsh
```

宿主机需要 NVIDIA 驱动、NVIDIA Container Toolkit、Docker、X11，以及 `python3`、`gh`、`git`、`ffmpeg`、`xdotool`、`xauth`。记录程序当前假设显示器为 `:0`。

原仿真容器使用 host 网络、host IPC、GPU，并将官方工作区挂载到 `/root/kuavo_ws`、X11 socket 挂载到 `/tmp/.X11-unix`。原策略容器使用 host 网络、GPU。在已导入官方镜像后，可创建新的策略容器再按第5节安装 Python/SDK：

```bash
sudo docker run -dit --name kuavo-ros-noetic-s12-ready --network host --gpus all kuavo_challenge_cup_2026:latest bash
```

若官方启动脚本给仿真容器分配了其他名字，使用 `docker ps` 确认新容器后将其改名为 `kuavo_challenge_container_GPU_046e4301`。不要覆盖已有同名容器。

不要把本机 `.config`、GitHub 令牌或旧 Xauthority 当作代码备份；新机器重新生成自己的显示授权。

## 5. 策略 Python/SDK 环境

策略容器须具有 ROS Noetic（`/opt/ros/noetic/setup.bash`），Miniforge 安装到 `/root/miniforge3`，策略环境路径固定为 `/root/miniforge3/envs/kls_dev`。Miniforge 官方来源：<https://github.com/conda-forge/miniforge>。

将恢复后的仓库复制到新策略容器 `/root/scene2-visual`，例如在宿主机仓库根目录执行 `sudo docker cp . kuavo-ros-noetic-s12-ready:/root/scene2-visual`，然后进入策略容器。在全新环境中执行：

```bash
cd /root/scene2-visual
bash recovery/install_algorithm_runtime.sh
```

此脚本依据 `conda-production-explicit.txt` 和 `pip-production-nodeps.txt` 重建原依赖，然后恢复捕获的 LeTools 和 SDK 代码。需要外部 conda/PyPI 下载源可访问；这不是 Docker 镜像备份。`--no-deps` 用于重建已记录的完整固定依赖集合，避免自动升级 NumPy；原有 OpenCV 的声明冲突如上所述，不隐瞒为干净依赖。

LeTools 上游为 <https://github.com/LejuRobotics/LeTools-Learning>，提交 `caf1892a3937c3bbbb075b6d88de8d8be6010116`。本附件额外保留实际修改的 `kuavo_deploy/src/scripts/script.py` 和两个部署 YAML。Scene2 调用机器人环境和 RGB-D，不运行 ACT 权重；配置中旧模型路径不意味着本策略需要加载该权重。

## 6. 恢复控制器、显示授权和运行入口

在两容器均无仿真/策略进程时，从宿主机仓库恢复 A/C：

```bash
sudo python3 vendor/scene1/tools/select_controller.py ac --apply
```

脚本验证 SHA-256，并拒绝覆盖未知控制器。若官方镜像更新导致 ABI 或旧文件哈希不一致，应按 `vendor/scene1/controllers/candidate-*` 中的源码/补丁重编译并重新验证，不可取消哈希保护。

X11 授权必须重新生成，例如在当前桌面会话中：

```bash
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f /tmp/scene2-restore.Xauthority nmerge -
sudo docker cp /tmp/scene2-restore.Xauthority kuavo_challenge_container_GPU_046e4301:/tmp/scene1-current-display.Xauthority
```

`tools/workstation.py` 本身固定使用 `DISPLAY=:0`；其他显示编号需要先适配工具。不要上传 Xauthority 文件。

当前 `tools/workstation.py` 调用 `sudo -n docker`，因此宿主机须允许该 Docker 命令免密执行，或在有权限且保留桌面显示环境的 root 会话运行工具。这里记录原工具的实际前提，不会自动修改系统 sudo 配置。

确认容器名和环境齐全后，在宿主机仓库：

```bash
mkdir -p evidence archives reports
python3 tools/workstation.py sync
python3 tools/workstation.py start --seed 0 --label restore-preflight
python3 tools/workstation.py run --mode preflight --label restore-preflight-check
python3 tools/workstation.py stop
```

上述 preflight 不运行抓取策略。完整重新验收使用新的唯一标签：

```bash
python3 tools/acceptance.py --phase acceptance --label restore-fresh-acceptance
```

当前 C 箱边界判定问题尚未修复。任何源码变更均产生新候选，不得沿用本次冻结源码的验证结论。
