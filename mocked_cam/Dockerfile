FROM mrnr91/uvicorn-gunicorn-fastapi:python3.8

WORKDIR /app/

# We use Poetry for dependency management
RUN curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | POETRY_HOME=/opt/poetry python && \
    cd /usr/local/bin && \
    ln -s /opt/poetry/bin/poetry && \
    poetry config virtualenvs.create false

# Copy poetry.lock* in case it doesn't exist in the repo
COPY ./pyproject.toml ./poetry.lock* /app/

RUN bash -c "poetry install --no-root"

COPY ./*.py /app
ENV PYTHONPATH=/app

EXPOSE 80
