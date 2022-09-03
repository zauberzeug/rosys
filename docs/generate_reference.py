import dataclasses
import importlib
import inspect
import logging
from pathlib import Path
from types import ModuleType

import mkdocs_gen_files

nav = mkdocs_gen_files.Nav()


def extract_events(filepath: str) -> dict[str, str]:
    with open(filepath, 'r') as f:
        lines = f.read().splitlines()
    events = {}
    for l, line in enumerate(lines):
        if line.endswith('= Event()'):
            event_name = line.strip().split()[0].removeprefix('self.')
            event_doc = lines[l+1].split("'''")[1]
            events[event_name] = event_doc
    return events


for path in sorted(Path('.').rglob('__init__.py')):
    identifier = str(path.parent).replace('/', '.')
    if identifier in ['rosys', 'rosys.test', 'rosys.hardware.communication']:
        continue

    try:
        module = importlib.import_module(identifier)
    except:
        logging.exeception(f'Failed to import {identifier}')
        continue

    doc_path = path.parent.with_suffix('.md')
    found_something = False
    for name in dir(module):
        if name.startswith('_'):
            continue  # skip private fields
        cls = getattr(module, name)
        if isinstance(cls, ModuleType):
            continue  # skip sub-modules
        if dataclasses.is_dataclass(cls):
            continue  # skip dataclasses
        if not cls.__doc__:
            continue  # skip classes without docstring
        events = extract_events(inspect.getfile(cls))
        with mkdocs_gen_files.open(Path('reference', doc_path), 'a') as fd:
            print(f'::: {identifier}.{name}', file=fd)
            if events:
                print('    options:', file=fd)
                print('      filters:', file=fd)
                for event_name in events:
                    print(f'        - "!{event_name}"', file=fd)
                print('### Events', file=fd)
                print('Name | Description', file=fd)
                print('- | -', file=fd)
                for event_name, event_doc in events.items():
                    print(f'{event_name} | {event_doc}', file=fd)
                print('', file=fd)
        found_something = True

    if found_something:
        nav[path.parent.parts[1:]] = doc_path.as_posix()

with mkdocs_gen_files.open('reference/SUMMARY.md', 'w') as nav_file:
    nav_file.writelines(nav.build_literate_nav())
