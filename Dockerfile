FROM python:3.10-buster

RUN apt update && apt install -y \
    sudo vim less ack-grep rsync wget curl cmake iproute2 iw python3-pip python3-autopep8 libgeos-dev graphviz graphviz-dev v4l-utils psmisc sysstat \
    libgl1-mesa-glx ffmpeg libsm6 libxext6 \
    avahi-utils iputils-ping \ 
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

# We use Poetry for dependency management
RUN curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | POETRY_HOME=/opt/poetry python && \
    cd /usr/local/bin && \
    ln -s /opt/poetry/bin/poetry && \
    poetry config virtualenvs.create false

WORKDIR /rosys

# only copy poetry package specs to minimize rebuilding of image layers
COPY pyproject.toml poetry.lock ./
RUN poetry config experimental.new-installer false

# Allow installing dev dependencies to run tests
ARG INSTALL_DEV=false
RUN bash -c "if [ $INSTALL_DEV == 'true' ] ; then poetry install -vvv --no-root ; else poetry install -vvv --no-root --no-dev ; fi"

COPY LICENSE README.md rosys.code-workspace ./
ADD ./rosys /rosys/rosys

ENV PYTHONPATH "${PYTHONPATH}:/rosys"

EXPOSE 8080

WORKDIR /app/
COPY ./start.sh /
COPY ./main.py /app/

# to simplify development within the RoSys container we preinstall vs code server and extensions
RUN curl -sSL https://gist.githubusercontent.com/b01/0a16b6645ab7921b0910603dfb85e4fb/raw/5186ea07a06eac28937fd914a9c8f9ce077a978e/download-vs-code-server.sh | sed "s/server-linux-x64/server-linux-$(dpkg --print-architecture)/" | sed "s/amd64/x64/" | bash
ENV VSCODE_SERVER=/root/.vscode-server/bin/*/server.sh
RUN $VSCODE_SERVER --install-extension ms-python.vscode-pylance \
    $VSCODE_SERVER --install-extension ms-python.python \
    $VSCODE_SERVER --install-extension himanoa.python-autopep8 \
    $VSCODE_SERVER --install-extension esbenp.prettier-vscode \
    $VSCODE_SERVER --install-extension littlefoxteam.vscode-python-test-adapter

CMD /start.sh
