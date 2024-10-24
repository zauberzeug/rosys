FROM python:3.12-bookworm

RUN apt update && apt install -y \
    sudo vim less ack-grep rsync wget curl cmake arp-scan iproute2 iw python3-pip python3-autopep8 libgeos-dev graphviz graphviz-dev v4l-utils psmisc sysstat \
    libgl1-mesa-glx ffmpeg libsm6 libxext6 \
    avahi-utils iputils-ping \
    jq \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

# We use Poetry for dependency management
RUN curl -sSL https://install.python-poetry.org | python3 - && \
    cd /usr/local/bin && \
    ln -s ~/.local/bin/poetry && \
    poetry config virtualenvs.create false


# only copy poetry package specs to minimize rebuilding of image layers
WORKDIR /rosys
COPY pyproject.toml ./

# Allow installing dev dependencies to run tests
ARG INSTALL_DEV=true
RUN bash -c "if [ $INSTALL_DEV == 'true' ] ; then poetry install -vvv --no-root --with dev ; else poetry install -vvv --no-root ; fi"

# Fetch Lizard firmware + scripts for hardware control
WORKDIR /root/.lizard
RUN CURL="curl -s https://api.github.com/repos/zauberzeug/lizard/releases" && \
    ZIP=$(eval "$CURL/latest" | jq -r '.assets[0].id') && \
    eval "$CURL/assets/$ZIP -LJOH 'Accept: application/octet-stream'" && \
    unzip *zip && \
    rm *zip && \
    ls -lha

# for Lizard monitor
RUN pip install --no-cache prompt-toolkit

WORKDIR /rosys
COPY LICENSE README.md rosys.code-workspace ./
ADD ./rosys /rosys/rosys
RUN poetry install -vvv

ENV PYTHONPATH="/rosys"

EXPOSE 8080

WORKDIR /app/
COPY ./start.sh /
COPY ./main.py /app/

CMD ["/start.sh"]
