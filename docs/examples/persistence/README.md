# Persistence

The `Persistable` class is a mixin for objects that need to be persisted.
It provides a `persistent` method that can be used to make the object persistent.

Derived classes must implement the `backup_to_dict` and `restore_from_dict` methods.

Note that the persistence module contains a number of helper functions:

- `to_dict`: converts (dictionaries or lists of) dataclasses into a dictionary (or list)
- `from_dict`: converts a dictionary into a dataclass of given type
- `replace_dict`: replaces the content of a dictionary using `from_dict` for each item
- `replace_list`: replaces the content of a list using `from_dict` for each item
- `replace_set`: replaces the content of a set using `from_dict` for each item
- `replace_dataclass`: replaces the attributes of a dataclass with the values of a dictionary

Further helper functions can be used for importing and exporting all persistable objects:

- The `export_all` method can be used to export all persistable objects to a dictionary.
- The `import_all` method can be used to import all persistable objects from a dictionary.
- Likewise, `export_button` and `import_button` are UI elements based on these two methods that can be added to a page.

By default, data is stored in the `~/.rosys` directory.
The filename is derived from the module name.
Both can be changed by setting the `path` and `key` parameters of the `persistent` method.

If you want to automatically keep daily backups, you can use the `BackupSchedule` module.
It will backup all the contents of your ~/.rosys directory at a configurable directory and at a given time each day.
When a maximum number of backup files is reached (specified with `backup_count`), it will delete the oldest file.

You should choose wisely which values to persist.
In particular, avoid to persist volatile things like wheel odometry or other sensor readings to save CPU and IO bandwidth.

```python
{! examples/persistable/main.py !}
```

## Migration from the old `PersistentModule` class

The `PersistentModule` class has been replaced by the `Persistable` mixin in version 0.24.0.
To migrate, follow these steps:

1. Replace `PersistentModule` with `Persistable` in the class definition.
2. Rename the `backup` and `restore` methods to `backup_to_dict` and `restore_from_dict`.
3. Use the `persistent` method to make objects persistent which used to be derived from `PersistentModule`:
   `KpiLogger`, `Schedule`, `PathPlanner`, `CameraProvider` and derived classes.
4. If you called `PersistentModule` with a `persistence_key`,
   remove it and use the `key` parameter of the `persistent` method instead.
