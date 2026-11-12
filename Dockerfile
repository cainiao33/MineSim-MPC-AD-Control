ARG BASE_IMAGE=python:3.10-slim
FROM ${BASE_IMAGE}

ARG APP_UID=10001
ARG APP_GID=10001
ARG INSTALL_REQUIREMENTS=true

LABEL org.opencontainers.image.title="mine-sim V1.1 contestant"
LABEL org.opencontainers.image.description="Contestant full-simulation runtime"
LABEL com.mine-sim.submission.version="1.1"

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /workspace/mine-sim

COPY setup/requirements.txt /tmp/requirements.txt
RUN if [ "${INSTALL_REQUIREMENTS}" = "true" ]; then \
      python -m pip install --no-cache-dir --upgrade pip \
      && python -m pip install --no-cache-dir -r /tmp/requirements.txt; \
    fi

RUN groupadd --gid "${APP_GID}" contestant \
    && useradd --uid "${APP_UID}" --gid "${APP_GID}" --create-home contestant

COPY --chown=contestant:contestant . .

RUN mkdir -p outputs \
    && chown -R contestant:contestant /workspace/mine-sim \
    && python -c "import matplotlib, numpy, pandas, PIL, scipy, shapely" \
    && python devkit/__main__.py --help >/dev/null

USER contestant

ENTRYPOINT ["python", "devkit/__main__.py"]
CMD ["--help"]
