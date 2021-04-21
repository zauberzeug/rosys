FROM mrnr91/uvicorn-gunicorn-fastapi:python3.7

RUN apt update && apt install openssh-server sudo vim less ack-grep rsync wget curl -y
# we configure the ssh server so we can remotely login via vscode for development
RUN sed -i 's/#Port 22/Port 5022/' /etc/ssh/sshd_config

COPY entrypoint.sh /docker-entrypoint.sh
ENTRYPOINT ["sh", "/docker-entrypoint.sh"]

WORKDIR /app/

# We use Poetry for dependency management
RUN curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | POETRY_HOME=/opt/poetry python && \
    cd /usr/local/bin && \
    ln -s /opt/poetry/bin/poetry && \
    poetry config virtualenvs.create false

# needed for opencv (wich is used to implement autofit)
RUN apt-get update; apt-get -y install libgl1-mesa-dev

# Copy poetry.lock* in case it doesn't exist in the repo
COPY ./system/pyproject.toml ./system/poetry.lock* /app/

# Allow installing dev dependencies to run tests
ARG INSTALL_DEV=false
RUN bash -c "if [ $INSTALL_DEV == 'true' ] ; then poetry install --no-root ; else poetry install --no-root --no-dev ; fi"

COPY ./system /app
ENV PYTHONPATH=/app

COPY authorized_keys /root/.ssh/

EXPOSE 80