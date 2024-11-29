from dataclasses_json import Exclude, config

from .backup_schedule import BackupSchedule
from .converters import from_dict, replace_dataclass, replace_dict, replace_list, replace_set, to_dict
from .persistent_module import PersistentModule
from .registry import backup, get_export, restore, restore_from_export, sync_backup, write_export
from .ui import export_button, import_button

exclude: dict[str, dict] = config(exclude=Exclude.ALWAYS)

__all__ = [
    'BackupSchedule',
    'PersistentModule',
    'backup',
    'exclude',
    'export_button',
    'from_dict',
    'get_export',
    'import_button',
    'replace_dataclass',
    'replace_dict',
    'replace_list',
    'replace_set',
    'restore',
    'restore_from_export',
    'sync_backup',
    'to_dict',
    'write_export',
]
