FROM mcr.microsoft.com/devcontainers/python:1-3.12-bookworm

# The Python base image adds a Yarn repository. This project does not use Yarn,
# so remove that source before updating the signed Debian package indexes.
RUN rm -f /etc/apt/sources.list.d/yarn.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends libegl1 libegl-mesa0 libgl1 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --no-cache-dir uv==0.12.22
