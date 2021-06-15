FROM mrnr91/uvicorn-gunicorn-fastapi:python3.8

RUN apt update && apt install openssh-server sudo vim less ack-grep rsync wget curl cmake iproute2 iw -y
# we configure the ssh server so we can remotely login via vscode for development
RUN sed -i 's/#Port 22/Port 5022/' /etc/ssh/sshd_config
COPY authorized_keys /root/.ssh/

COPY entrypoint.sh /docker-entrypoint.sh
ENTRYPOINT ["sh", "/docker-entrypoint.sh"]

WORKDIR /app/

# We use Poetry for dependency management
RUN curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | POETRY_HOME=/opt/poetry python && \
    cd /usr/local/bin && \
    ln -s /opt/poetry/bin/poetry && \
    poetry config virtualenvs.create false

# Copy poetry.lock* in case it doesn't exist in the repo
COPY ./pyproject.toml ./poetry.lock* /app/
RUN poetry config experimental.new-installer false
# Allow installing dev dependencies to run tests
ARG INSTALL_DEV=false

RUN bash -c "if [ $INSTALL_DEV == 'true' ] ; then poetry install -vvv --no-root ; else poetry install -vvv --no-root --no-dev ; fi"

# our demo in main.py requires NiceGui for visualization in the browser
RUN python3 -m pip install nicegui

ADD ./rosys /app/rosys
COPY  main.py start.sh /app/

ENV PYTHONPATH=/app

EXPOSE 80
