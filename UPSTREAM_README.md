# mine-sim 参赛选手开发指南

mine-sim 是矿区非结构化道路动态场景的自动驾驶回放仿真环境。参赛选手可以修改预测、规划和控制 baseline，通过仿真生成结果 CSV，并对结果进行可视化复盘和评分。

```text
安装环境 → 修改 baseline → 运行测试 → 查看 CSV/可视化 → 评分
```

以下命令均在项目根目录执行。

## 1. 环境安装

推荐使用 Python 3.9 或 3.10：

```bash
conda create -n mine_sim python=3.10
conda activate mine_sim
pip install -r setup/requirements.txt
```

查看命令行帮助和 `inputs/` 中的公开场景：

```bash
python devkit/__main__.py --help
python devkit/__main__.py --list-scenarios
```

## 2. 开发 baseline

参赛选手主要修改以下三个文件：

| 文件 | 职责 | 主接口 |
| --- | --- | --- |
| `devkit/planner.py` | 根据观测和目标区域规划路径 | `Planner.process(observation)` |
| `devkit/predictor.py` | 预测背景车辆未来轨迹 | `Predictor.predict(...)` |
| `devkit/simple_control.py` | 跟踪规划路径并生成控制量 | `MotionController.process(...)` |

仿真入口 `devkit/__main__.py` 直接调用这三个 baseline：

```text
场景初始化：Planner.process → path_planned
每个仿真步：Predictor.predict → MotionController.process → action
```

`action` 为 `(加速度, 前轮转角)`，仿真器将其限制在以下范围：

- 加速度：`[-15, 15] m/s²`；
- 前轮转角：`[-1, 1] rad`。

## 3. 运行仿真测试

### 3.1 运行单个公开场景

```bash
python devkit/__main__.py \
  --scenario Scenario-dapai_intersection_1_3_10.json \
  --visualize none
```

`--scenario` 支持场景文件名、场景名、通配符和 JSON 文件路径：

```bash
python devkit/__main__.py \
  --scenario 'dapai_intersection_1_3_*' \
  --max-scenes 2
```

场景名需以 `dapai_` 或 `jiangtong_` 开头，仿真器会根据前缀选择对应地图。

### 3.2 批量运行场景

将 `Scenario-*.json` 放入 `inputs/`，不传入 `--scenario` 即可运行目录内全部场景：

```bash
python devkit/__main__.py \
  --skip-existing \
  --continue-on-error \
  --visualize none
```

常用参数：

| 参数 | 含义 |
| --- | --- |
| `--inputs DIR` | 待测场景目录，默认为 `inputs/` |
| `--datasets DIR` | 地图数据目录，默认为 `datasets/` |
| `--outputs DIR` | 结果目录，默认为 `outputs/` |
| `--visualize none` | 关闭绘图 |
| `--visualize show` | 实时显示仿真画面 |
| `--visualize save` | 保存逐帧 PNG，不显示窗口 |
| `--skip-existing` | 跳过已存在结果 CSV 的场景 |
| `--continue-on-error` | 单场景失败后继续运行其余场景 |
| `--max-scenes N` | 限制本次运行的场景数 |

## 4. 测试输出

每个场景生成 `<场景名>_result.csv`，每次运行同时生成 `run_summary.csv`：

```text
outputs/
├── <场景名>_result.csv
└── run_summary.csv
```

结果 CSV 第一列是仿真时间，随后按车辆记录：

```text
x, y, yaw_rad, v_mps, acc_mpss,
width, length, locationPoint2Head, locationPoint2Rear
```

字段带车辆后缀，例如 `x_ego`、`x_1`。末列 `end` 表示场景终止状态：

| `end` | 含义 |
| ---: | --- |
| -1 | 仿真未结束或算法异常中断 |
| 1 | 到达场景最大时间 |
| 2 | 自车与背景车碰撞 |
| 3 | 自车与道路边界碰撞 |
| 4 | 自车到达目标区域 |

`run_summary.csv` 汇总场景终止码、步数、仿真时间、程序运行时间、结果路径和异常信息。

## 5. 可视化调试

仿真器提供在线逐帧可视化和 CSV 离线复盘两种方式：

| 方式 | 数据来源 | 用途 | 特点 |
| --- | --- | --- | --- |
| 在线逐帧可视化 | 仿真内存中的观测、预测和真实轨迹 | 调试预测和规划内部过程 | 包含中间调试信息，需在仿真时绘图 |
| CSV 离线复盘 | 已生成的结果 CSV | 查看实际轨迹、速度和加速度 | 无需重新仿真，不包含 CSV 未记录的中间量 |

### 5.1 在线逐帧 PNG 和 GIF

运行仿真并保存逐帧 PNG：

```bash
python devkit/__main__.py \
  --scenario Scenario-dapai_intersection_1_3_10.json \
  --visualize save
```

图片保存在：

```text
outputs/visualizations/image_<场景名>/images/
```

`gif_generateor_simple.py` 负责将这些 PNG 帧合成 GIF：

```bash
python other_scripts/gif_generateor_simple.py \
  outputs/visualizations/image_dapai_intersection_1_3_10/images \
  --fps 10
```

### 5.2 CSV 离线轨迹图和 GIF

生成静态轨迹图：

```bash
python other_scripts/visualize_result.py \
  outputs/dapai_intersection_1_3_10_result.csv
```

生成动画 GIF：

```bash
python other_scripts/visualize_result.py \
  outputs/dapai_intersection_1_3_10_result.csv \
  --output outputs/dapai_intersection_1_3_10_result.gif \
  --stride 2 \
  --fps 10
```

`--stride` 用于降低长场景的 GIF 帧数，`--no-map` 用于关闭地图底图。

## 6. 结果评分

`score_result.py` 读取一个或多个结果 CSV，生成包含分项指标和综合分的 JSON 报告。公开场景的期望耗时保存在 `inputs/expected_times.json`，评分器默认读取该文件。

### 6.1 单场景评分

```bash
python other_scripts/score_result.py \
  outputs/dapai_intersection_1_3_10_result.csv
```

默认评分报告为：

```text
outputs/dapai_intersection_1_3_10_result_score.json
```

### 6.2 批量评分

`inputs/expected_times.json` 是“场景名→期望耗时”的 JSON 对象。对结果目录中的全部 `*_result.csv` 评分：

```bash
python other_scripts/score_result.py outputs/ \
  --output outputs/score_report.json
```

如需使用其他期望耗时文件，可显式传入 `--expected-times FILE.json`；使用统一耗时时可传入 `--expected-time SECONDS`。

批量报告的综合分为各场景总分的算术平均。

### 6.3 评分规则

满分 100 分，由安全性 60 分和效率性 40 分组成。

安全性：

- 与背景车或道路边界碰撞：安全分为 0；
- 前车 TTC 低于 3 s：根据违规持续时间占比，从 20 分中扣除；
- 侧向车辆外轮廓距离低于 0.8 m：根据违规持续时间占比，从 20 分中扣除；
- 侧向加速度绝对值超过 0.4 m/s²：根据违规持续时间占比，从 20 分中扣除。

效率性：

- 到达目标区域得 20 分，否则得 0 分；
- 耗时分为 `min(期望耗时 / 实际耗时, 1) × 20`。

结果不完整、末帧为 `end=-1` 或缺少关键自车字段时，评分结果为 0。

## 7. 推荐开发流程

1. 运行内置 baseline，确认环境和数据可用；
2. 修改预测、规划或控制模块；
3. 使用单场景进行快速测试；
4. 使用 CSV 离线图查看轨迹和车辆状态；
5. 需要中间预测或规划信息时，使用在线逐帧可视化；
6. 批量运行场景并使用统一参数评分。

## 8. 项目自测

```bash
python -m unittest discover -s tests -v
```

## 9. 目录结构参考

公开 Demo 包建议采用以下目录结构：

```text
mine_sim_V1.1_local/
├── datasets/                              # 仅包含地图数据
│   └── maps/
│       ├── bitmap/                        # 可行驶区域掩码和地图底图
│       │   ├── guangdong_dapai_bitmap_mask.png
│       │   ├── guangdong_dapai_bitmap_rgb.png
│       │   ├── jiangxi_jiangtong_bitmap_mask.png
│       │   └── jiangxi_jiangtong_bitmap_rgb.png
│       ├── semantic_map/                  # 道路、路口和参考路径等语义地图
│       │   ├── guangdong_dapai_semantic_map.json
│       │   ├── guangdong_dapai_semantic_map-v1.6.json
│       │   ├── jiangxi_jiangtong_semantic_map.json
│       │   ├── jiangxi_jiangtong_semantic_map-v1.5.json
│       │   └── local_origin_info.py       # 地图局部坐标原点信息
│       └── other_figure/                  # 航拍图、边界图和地图可视化资料
│           ├── guangdong_dapai_aerialpicture_*.JPG
│           ├── guangdong_dapai_borderline_2D.fig
│           ├── jiangxi_jiangtong_aerialpicture.jpg
│           └── jiangxi_jiangtong_borderline_*.fig
├── devkit/                                # 仿真环境和参赛算法
│   ├── __main__.py                        # 仿真命令行入口
│   ├── mine_sim_test_conf.py              # 默认测试配置
│   ├── planner.py                         # 规划 baseline，参赛者可修改
│   ├── predictor.py                       # 预测 baseline，参赛者可修改
│   ├── simple_control.py                  # 控制 baseline，参赛者可修改
│   ├── common/                            # 通用数学和数据结构工具
│   ├── dynamic_scenes/                    # 场景回放、车辆更新、记录和在线绘图
│   └── map_expansion/                     # 地图读取、查询和渲染
│
├── inputs/                                # 待测试场景
│   ├── expected_times.json                # 公开场景期望耗时
│   ├── Scenario-dapai_intersection_*.json
│   └── Scenario-jiangtong_intersection_*.json
│
├── outputs/                               # 仿真运行自动生成的结果目录（初始为空）
│   ├── <场景名>_result.csv
│   ├── run_summary.csv
│   └── visualizations/
│
├── other_scripts/                         # 结果处理工具
│   ├── visualize_result.py                # CSV 静态图和 GIF 复盘
│   ├── gif_generateor_simple.py           # 将在线绘图帧合成为 GIF
│   ├── score_result.py                    # 结果评分
│   ├── mine-sim-v1.1-runtime-amd64.tar    # 已预装依赖的离线基础镜像，请勿删除或改名
│   └── build_submission_image.sh          # 参赛镜像一键构建脚本
│
├── setup/
│   ├── requirements.txt                   # 仿真器官方 Python 依赖
│   └── requirements-extra.txt             # 选手可选的额外 Python 依赖
├── Dockerfile                             # 参赛镜像定义
├── .dockerignore
├── LICENSE
└── README.md
```

其中：

- `inputs/` 包含 40 个公开场景和 `expected_times.json`；程序只把 `Scenario-*.json` 作为场景；
- `datasets/maps/` 是规划、道路边界检测和可视化所必需的公共资源，需要保留；
- 场景 JSON 已包含回放所需的完整时序状态，不再依赖 `datasets/Scenarios/` 或 `datasets/TrajDataResults/`；
- `outputs/` 为仿真输出目录，初次下载为空，运行仿真后会自动在本地生成结果文件。

## 10. Docker 打包与运行

### 10.1 打包前准备

打包电脑需要安装并启动 Docker。请保持下面两个文件位于 `other_scripts/` 中，不要删除、移动或改名：

```text
other_scripts/build_submission_image.sh
other_scripts/mine-sim-v1.1-runtime-amd64.tar
```

其中 `mine-sim-v1.1-runtime-amd64.tar` 是比赛提供的离线基础镜像，已经安装仿真所需的官方 Python 依赖。没有额外依赖时，打包过程不会执行 `pip install`，也不需要连接 PyPI。

如果算法还需要官方基础镜像未提供的 Python 包，请将其逐行填写到 `setup/requirements-extra.txt`，并建议固定版本，例如：

```text
requests==2.32.3
```

不需要额外依赖时，请保持该文件为空或只保留注释。打包脚本会自动检查：没有有效依赖行时完全跳过 `pip install`；存在有效依赖行时，只安装该文件声明的增量 Python 包。额外依赖需要兼容 Python 3.10 和 `linux/amd64`，其下载时间和占用空间会计入最终提交镜像。

### 10.2 一键生成上传文件

完成 `devkit/planner.py`、`devkit/predictor.py` 或 `devkit/simple_control.py` 的算法开发后，在 `mine_sim_V1.1_local` 项目根目录执行：

```bash
./other_scripts/build_submission_image.sh
```

脚本不需要参数，会自动完成以下工作：

1. 从 `other_scripts/mine-sim-v1.1-runtime-amd64.tar` 加载离线基础镜像；
2. 构建固定为 `linux/amd64` 的参赛镜像；
3. 检查镜像入口，并确认其中包含公开测试场景；
4. 在项目根目录生成网站上传文件。

最终输出为：

```text
mine-sim-v1.1-submission.tar
```

再次执行脚本会安全地覆盖之前生成的上传文件。请上传最终的 `mine-sim-v1.1-submission.tar`，不要上传基础镜像，也不要自行压缩整个源码目录。

该文件是经过快速压缩的 Docker archive，虽然保留 `.tar` 文件名，但 Docker 可以直接识别并加载。可通过下面的命令检查文件是否能够加载：

```bash
docker load -i mine-sim-v1.1-submission.tar
```

加载后的镜像名称固定为：

```text
mine-sim-v1.1-local:demo
```

### 10.3 在本机运行参赛镜像

运行单个公开场景并将结果写到宿主机目录：

```bash
mkdir -p docker_outputs
chmod 777 docker_outputs
docker run --rm \
  --mount type=bind,src="$(pwd)/docker_outputs",dst=/workspace/mine-sim/outputs \
  mine-sim-v1.1-local:demo \
  --scenario Scenario-dapai_intersection_1_3_10.json \
  --visualize none
```

结果 CSV 将写入 `docker_outputs/`。如果脚本提示无法连接 Docker，请先启动 Docker 服务，并确认当前用户具有 Docker 使用权限；如果导出阶段失败，请确认磁盘留有足够的临时空间。

### 10.4 常见问题

#### Q1：构建过程中报错 `groupadd: group 'contestant' already exists`

基础镜像 `mine-sim-v1.1-runtime:1.1` 已创建`contestant` 用户，而脚本尝试重复创建同名用户导致失败。

1. 打开 `other_scripts/build_submission_image.sh`，找到以下两行：

```dockerfile
RUN groupadd --gid 10001 contestant \
    && useradd --uid 10001 --gid 10001 --create-home contestant
```

2. 在每行开头添加 `#` 注释掉，修改为：

```dockerfile
# RUN groupadd --gid 10001 contestant \
#     && useradd --uid 10001 --gid 10001 --create-home contestant
```

此操作仅跳过用户创建步骤，直接使用基础镜像中已有的 `contestant` 用户，不影响镜像功能。

#### Q2：提示 `错误：Docker 服务未启动`

Docker 服务未运行，或当前用户没有 Docker 使用权限。
