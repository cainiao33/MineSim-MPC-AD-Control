#!/usr/bin/env bash
# 参赛选手一键打包脚本：把当前 mine_sim_V1.1_local 构建成网站上传用 Docker tar。
set -Eeuo pipefail

if [[ $# -gt 0 ]]; then
  if [[ "${1}" == "-h" || "${1}" == "--help" ]]; then
    echo "用法：./other_scripts/build_submission_image.sh"
    echo "输出：mine-sim-v1.1-submission.tar"
    exit 0
  fi
  echo "错误：本脚本不需要参数，请直接执行：./other_scripts/build_submission_image.sh" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
image_ref="mine-sim-v1.1-local:demo"
runtime_image_ref="mine-sim-v1.1-runtime:1.1"
runtime_archive="${script_dir}/mine-sim-v1.1-runtime-amd64.tar"
output_archive="${project_root}/mine-sim-v1.1-submission.tar"
platform="linux/amd64"

required_paths=(
  "setup/requirements.txt"
  "setup/requirements-extra.txt"
  "other_scripts/mine-sim-v1.1-runtime-amd64.tar"
  "devkit/__main__.py"
  "devkit/planner.py"
  "devkit/predictor.py"
  "devkit/simple_control.py"
  "datasets/maps/bitmap/jiangxi_jiangtong_bitmap_mask.png"
  "datasets/maps/bitmap/guangdong_dapai_bitmap_mask.png"
  "datasets/maps/semantic_map/jiangxi_jiangtong_semantic_map.json"
  "datasets/maps/semantic_map/guangdong_dapai_semantic_map.json"
  "inputs/expected_times.json"
)
for relative_path in "${required_paths[@]}"; do
  if [[ ! -e "${project_root}/${relative_path}" ]]; then
    echo "错误：local 工程缺少 ${relative_path}" >&2
    exit 1
  fi
done
if ! compgen -G "${project_root}/inputs/Scenario-*.json" >/dev/null; then
  echo "错误：inputs/ 中没有公开场景 Scenario-*.json。" >&2
  exit 1
fi

command -v docker >/dev/null 2>&1 || {
  echo "错误：没有安装 Docker，或 docker 不在 PATH 中。" >&2
  exit 1
}
docker info >/dev/null 2>&1 || {
  echo "错误：Docker 服务未启动，或当前用户没有 Docker 使用权限。" >&2
  exit 1
}
command -v gzip >/dev/null 2>&1 || {
  echo "错误：系统缺少 gzip。" >&2
  exit 1
}

temporary_dir="$(mktemp -d)"
temporary_archive="${output_archive}.partial"
temporary_raw_archive="${output_archive}.docker-save.partial"
cleanup() {
  rm -rf -- "${temporary_dir}"
  rm -f -- "${temporary_archive}"
  rm -f -- "${temporary_raw_archive}"
}
trap cleanup EXIT

# 只为上传镜像生成临时 Dockerfile，不修改 local 中的开发 Dockerfile。
# 官方 Python 依赖已经放在 runtime 基础镜像中；只有声明额外依赖时才会运行 pip。
# 镜像只复制云端无界面仿真实际需要的文件，不复制参考大图、重复地图、tests 和 outputs。
submission_dockerfile="${temporary_dir}/Dockerfile.submission"
cat >"${submission_dockerfile}" <<'DOCKERFILE'
FROM mine-sim-v1.1-runtime:1.1

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /workspace/mine-sim

COPY setup/requirements-extra.txt /tmp/requirements-extra.txt
RUN if grep -Eq '^[[:space:]]*[^#[:space:]]' /tmp/requirements-extra.txt; then \
      echo "正在安装选手声明的额外 Python 依赖..."; \
      python -m pip install --no-cache-dir \
        --index-url https://mirrors.aliyun.com/pypi/simple/ \
        -r /tmp/requirements-extra.txt; \
    else \
      echo "未声明额外 Python 依赖，跳过 pip install。"; \
    fi

RUN groupadd --gid 10001 contestant \
    && useradd --uid 10001 --gid 10001 --create-home contestant

COPY --chown=contestant:contestant devkit/ devkit/
COPY --chown=contestant:contestant inputs/ inputs/
COPY --chown=contestant:contestant datasets/maps/bitmap/*_bitmap_mask.png datasets/maps/bitmap/
COPY --chown=contestant:contestant datasets/maps/semantic_map/*_semantic_map.json datasets/maps/semantic_map/

RUN mkdir -p outputs \
    && chown -R contestant:contestant /workspace/mine-sim \
    && python -c "import matplotlib, numpy, pandas, PIL, scipy, shapely" \
    && python devkit/__main__.py --help >/dev/null

USER contestant
ENTRYPOINT ["python", "devkit/__main__.py"]
CMD ["--help"]
DOCKERFILE

echo "[1/4] 正在准备离线运行环境..."
if docker image inspect "${runtime_image_ref}" >/dev/null 2>&1; then
  echo "      已存在基础镜像：${runtime_image_ref}"
else
  echo "      首次使用，正在加载：${runtime_archive}"
  docker load --input "${runtime_archive}" >/dev/null
  if ! docker image inspect "${runtime_image_ref}" >/dev/null 2>&1; then
    echo "错误：基础镜像加载后未找到 ${runtime_image_ref}。" >&2
    exit 1
  fi
fi

echo "[2/4] 正在构建参赛镜像..."
echo "      项目：${project_root}"
echo "      镜像：${image_ref}"
echo "      平台：${platform}"
docker build \
  --platform "${platform}" \
  --file "${submission_dockerfile}" \
  --tag "${image_ref}" \
  "${project_root}"

echo "[3/4] 正在检查参赛镜像..."
scenario_count="$(docker run --rm --platform "${platform}" "${image_ref}" --list-scenarios | wc -l)"
if [[ "${scenario_count}" -le 0 ]]; then
  echo "错误：镜像中没有可运行的公开场景。" >&2
  exit 1
fi
echo "      自检通过：发现 ${scenario_count} 个公开场景。"

echo "[4/4] 正在生成网站上传文件..."
docker save --output "${temporary_raw_archive}" "${image_ref}"
if command -v pigz >/dev/null 2>&1; then
  pigz -1 --stdout "${temporary_raw_archive}" >"${temporary_archive}"
else
  gzip -1 --stdout "${temporary_raw_archive}" >"${temporary_archive}"
fi
if [[ ! -s "${temporary_archive}" ]]; then
  echo "错误：生成的上传文件为空。" >&2
  exit 1
fi
rm -f -- "${temporary_raw_archive}"
mv -f -- "${temporary_archive}" "${output_archive}"

archive_size="$(du -h "${output_archive}" | awk '{print $1}')"
echo
echo "打包完成：${output_archive}"
echo "文件大小：${archive_size}"
echo "该文件可直接上传；云侧使用 docker load -i 加载。"
