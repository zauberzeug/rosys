import dataclasses
import importlib
from pathlib import Path
from types import ModuleType

import mkdocs_gen_files

nav = mkdocs_gen_files.Nav()

for path in sorted(Path('.').rglob('__init__.py')):
    identifier = str(path.parent).replace('/', '.')
    if identifier in ['rosys', 'rosys.test', 'rosys.hardware.communication']:
        continue

    try:
        module = importlib.import_module(identifier)
    except:
        continue

    doc_path = path.parent.with_suffix('.md')
    found_something = False
    for name in dir(module):
        if name.startswith('_'):
            continue  # skip private fields
        obj = getattr(module, name)
        if isinstance(obj, ModuleType):
            continue  # skip sub-modules
        if dataclasses.is_dataclass(obj):
            continue  # skip dataclasses
        if not obj.__doc__:
            continue  # skip classes without docstring
        with mkdocs_gen_files.open(Path('reference', doc_path), 'a') as fd:
            print(f'::: {identifier}.{name}', file=fd)
        found_something = True

    if found_something:
        nav[path.parent.parts[1:]] = doc_path.as_posix()

with mkdocs_gen_files.open('reference/SUMMARY.md', 'w') as nav_file:
    nav_file.writelines(nav.build_literate_nav())
