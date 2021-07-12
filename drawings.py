import os

path = '/data/drawings'
if not os.path.exists(path):
    os.makedirs(path)


def store(content: bytearray):

    with open(f'{path}/default.svg', 'wb') as f:
        f.write(content)
