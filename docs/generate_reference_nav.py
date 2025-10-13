import dataclasses
import importlib
import inspect
import logging
import re
import sys
from pathlib import Path
from types import ModuleType

import mkdocs_gen_files

nav = mkdocs_gen_files.Nav()


def extract_events(filepath: str) -> dict[str, str]:
    with open(filepath) as f:
        lines = f.read().splitlines()
    events_: dict[str, str] = {}
    for l, line in enumerate(lines):
        if re.search(r'= Event(\[.*?\])?\(\)$', line):
            event_name_ = line.strip().split()[0].removeprefix('self.')
            event_doc_ = lines[l+1].split('"""')[1]
            events_[event_name_] = event_doc_
    return events_


for path in sorted(Path('rosys').rglob('__init__.py')):
    identifier = str(path.parent).replace('/', '.')
    if identifier in ['rosys', 'rosys.testing', 'rosys.hardware.communication', 'rosys.persistence']:
        continue

    try:
        module = importlib.import_module(identifier)
    except Exception:
        logging.exception('Failed to import %s', identifier)
        sys.exit(1)

    doc_path = path.parent.with_suffix('.md')
    found_something = False
    for name in getattr(module, '__all__', dir(module)):
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
