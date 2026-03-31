FROM python:3.13-bookworm

RUN apt update && apt install -y \
    sudo vim less ack-grep rsync wget curl cmake arp-scan iproute2 iw python3-pip python3-autopep8 libgeos-dev graphviz graphviz-dev v4l-utils psmisc sysstat \
    libgl1 libglx-mesa0 ffmpeg libsm6 libxext6 libturbojpeg0 \
    avahi-utils iputils-ping \
    jq \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*

# Install uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# Only copy dependency specs to minimize rebuilding of image layers
WORKDIR /rosys
COPY pyproject.toml uv.lock ./

# Allow installing dev dependencies to run tests
ARG INSTALL_DEV=true
RUN if [ "$INSTALL_DEV" = "true" ]; then \
      uv sync --no-install-project; \
    else \
      uv sync --no-install-project --no-dev; \
    fi

# Fetch Lizard firmware + scripts for hardware control
WORKDIR /root/.lizard
RUN CURL="curl -s https://api.github.com/repos/zauberzeug/lizard/releases" && \
    ZIP=$(eval "$CURL/latest" | jq -r '.assets[0].id') && \
    eval "$CURL/assets/$ZIP -LJOH 'Accept: application/octet-stream'" && \
    unzip *zip && \
    rm *zip && \
    ls -lha
RUN uv pip install --python /rosys/.venv/bin/python --no-cache -r requirements.txt

WORKDIR /rosys
COPY LICENSE README.md rosys.code-workspace ./
ADD ./rosys /rosys/rosys
ARG VERSION=0.0.0
ENV POETRY_DYNAMIC_VERSIONING_BYPASS=$VERSION
RUN uv sync

ENV PATH="/rosys/.venv/bin:$PATH"
ENV PYTHONPATH="/rosys"

EXPOSE 8080

WORKDIR /app/
COPY ./start.sh /
COPY ./main.py /app/

CMD ["/start.sh"]
