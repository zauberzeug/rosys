FROM python:3.9-buster

RUN apt update && apt install sudo libcurl4-openssl-dev libssl-dev vim less ack-grep rsync wget curl cmake iproute2 iw -y && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

# We use Poetry for dependency management
RUN curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | POETRY_HOME=/opt/poetry python && \
    cd /usr/local/bin && \
    ln -s /opt/poetry/bin/poetry && \
    poetry config virtualenvs.create false

WORKDIR /app/

# Copy poetry.lock* in case it doesn't exist in the repo
COPY ./pyproject.toml ./poetry.lock* ./
RUN poetry config experimental.new-installer false
# Allow installing dev dependencies to run tests
ARG INSTALL_DEV=false

RUN bash -c "if [ $INSTALL_DEV == 'true' ] ; then poetry install -vvv --no-root ; else poetry install -vvv --no-root --no-dev ; fi"

# our demo in main.py requires NiceGui for visualization in the browser
RUN python3 -m pip install 'nicegui==0.3.6'

ADD ./rosys /app/rosys
COPY  main.py start.sh /app/

EXPOSE 80

