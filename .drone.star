TAG='${DRONE_BRANCH}-${DRONE_COMMIT_SHA:0:8}'

def main(ctx):
    return [
        pipeline(name='Testing', branches=['master'], steps=[
            basic(name='prepare', image='mikefarah/yq:4', cmds=[
                # port 80 is occupied by drone, so we tell linux to seek any free port by setting "0"
                'yq eval \'(.services.proxy.ports[0] = "0:80")\' docker-compose.yml -i',

                # docker in docker makes it tricky to mount volumes (see https://discourse.drone.io/t/cannot-figure-out-how-to-mount-volume/3808/2)
                # but because they are not needed for testing, we just remove the entries
                'yq eval "del(.services.db.volumes)" docker-compose.yml -i',
                'yq eval "del(.services.system.volumes)" docker-compose.yml -i',
                'yq eval "del(.services.frontend.volumes)" docker-compose.yml -i',

                # remove ports to not expose internal services (see https://askubuntu.com/a/652572/24080)
                'yq eval "del(.services.system.ports)" docker-compose.yml -i',
                'yq eval "del(.services.frontend.ports)" docker-compose.yml -i',

                # use real uvicorn call as later in deployment (debugpy somhow disturbs the tests)
                'yq eval \'(.services.backend.command = "/app/start.sh")\' docker-compose.yml -i',
                'yq eval \'(.services.*trainer*.command = "/app/start.sh")\' docker-compose.yml -i',
            ]),
            docker_compose(name='build', cmds=[
                'docker-compose -p robot-brain rm -f -v', 
                'docker-compose -p robot-brain up -d --build',
            ]),
            docker_compose(name='pytest', cmds='docker-compose -p robot-brain exec -T backend pytest -s -v'),
            docker_compose(name='cleanup', cmds='docker-compose -p robot-brain stop', when={'status': ['success', 'failure']}),
            docker_compose(name='analyze', cmds='docker-compose -p robot-brain ps && docker-compose -p robot-brain logs', when={'status': ['failure']}),
            slack(webhook='https://hooks.slack.com/services/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'),
        ]),
    ]


def pipeline(name, branches, steps, depends_on=None):
    result = {
        'kind': 'pipeline',
        'name': name,
        'concurrency': {'limit': 1 },
        'steps': steps,
        'trigger': { 'event' : ['push'], 'branch' : branches },
        'volumes': [ # each pipeline is enabled to support docker-compose by providing the neccessary volume infos
            {'host': { 'path': '/var/run/docker.sock'}, 'name': 'docker_socket' }, 
        ],
    }
    if depends_on != None: 
        result.update({'depends_on': [depends_on]})

    return result

def basic(name, image, cmds, when=None):
    if type(cmds) == 'string':
        cmds = [cmds]#.splitlines(keepends=False)
    return {
        'name': name,
        'image': image,
        'commands': cmds,
        'when': when,
    }

def docker_compose(name, cmds, when=None):
    result = basic(name, 'docker/compose:alpine-1.28.5', cmds, when)
    result.update({
        'privileged': True,
        'volumes' : [ # we need to provide volumes that allow docker-compose to run inside container
            {'name': 'docker_socket', 'path': '/var/run/docker.sock' },
            {'name': 'docker_config', 'path': '/root/.docker/config.json' },
        ],
    })
    return result

def slack(webhook):
    return {
        'image': 'plugins/slack:1.3.0', 
        'when': { 'status': ['failure'] }, 
        'name': 'slack', 
        'settings': { 'webhook': webhook }
  }