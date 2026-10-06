# syntax=docker/dockerfile:1
FROM ubuntu:22.04 AS sim
ARG DEBIAN_FRONTEND=noninteractive
ARG PX4_REF=v1.16.0
ARG BUILD_JOBS=4
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git sudo python3 python3-pip tini procps \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt
RUN git clone --branch ${PX4_REF} --depth 1 --recursive --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git PX4
WORKDIR /opt/PX4
RUN bash Tools/setup/ubuntu.sh --no-nuttx
COPY config/sitl.params /tmp/sitl.params
RUN cat /tmp/sitl.params >> ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500 \
    && make -j${BUILD_JOBS} px4_sitl_default \
    && cmake --build build/px4_sitl_default --target px4_gz_plugins -j${BUILD_JOBS} \
    && test -f build/px4_sitl_default/rootfs/gz_env.sh \
    && git rev-parse HEAD > /opt/PX4_COMMIT \
    && git submodule status --recursive > /opt/PX4_SUBMODULES \
    && dpkg-query -W > /opt/OS_PACKAGES
COPY swarm /app/swarm
COPY config/mission.json /app/config/mission.json
COPY scripts /app/scripts
ENV PYTHONPATH=/app GZ_PARTITION=three_drone_swarm GZ_IP=127.0.0.1 \
    LIBGL_ALWAYS_SOFTWARE=1
WORKDIR /app
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["bash", "/app/scripts/launch_sim.sh"]

FROM python:3.11-slim-bookworm AS companion
ENV PYTHONUNBUFFERED=1 PYTHONPATH=/app MAVLINK20=1
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt && pip freeze > /app/PYTHON_PACKAGES
COPY swarm /app/swarm
COPY scripts /app/scripts
COPY tests /app/tests
COPY config /app/config
CMD ["python", "-m", "swarm.coordinator"]
