# Persistence

RoSys' `PersistentModule` provides an easy interface to backup and restore parts of the object state.
The following example demonstrates a `Model` class that has `value`, which is manipulated with a `ui.slider`.

```python
{!src/example_persistence.py !}
```

By deriving from `PersistentModule` and implementing `backup` and `restore`, RoSys will automatically write the value to a file in the directory ~/.rosys/.
The filename contains the name of the module.
After restarting the script, the value will be restored to its last state.

The `request_backup` method can be called to enforce a backup within RoSys' next backup cycle, which happens every 10 seconds.
During shutdown, all backups are performed, independent of whether `request_backup` has been called.

The `backup` function can return any JSON-serializable dictionary that represents the current state.
It should match the `restore` function so that it can translate it back to object state.

You should choose wisely which values to persist.
Try to avoid consuming unnecessary CPU and IO bandwidth for volatile things like wheel odometry or other sensor readings.

Note that the persistence module contains a number of helper functions:

- `to_dict`: converts (dictionaries or lists of) dataclasses into a dictionary (or list)
- `from_dict`: converts a dictionary into a dataclass of given type
- `replace_dict`: replaces the content of a dictionary using `from_dict` for each item
- `replace_list`: replaces the content of a list using `from_dict` for each item
- `replace_set`: replaces the content of a set using `from_dict` for each item
- `replace_dataclass`: replaces the attributes of a dataclass with the values of a dictionary

The persistence module also provides UI buttons for exporting the contents of the ~/.rosys directory to a single file and for importing such a file.

If you want to automatically keep daily backups, you can use the `BackupSchedule` module.
It will backup all the contents of your ~/.rosys directory at a configurable directory and at a given time each day.
When a maximum number of backup files is reached (specified with `backup_count`), it will delete the oldest file.
