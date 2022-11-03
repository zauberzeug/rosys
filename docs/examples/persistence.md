# Persistence

RoSys' persistence module provides an easy interface to backup and restore parts of the object state.
The following example demonstrates a `Model` class that has `value`, which is manipulated with a `ui.slider`.

```python
{!src/example_persistence.py !}
```

By registering itself with the `persistence` module and implementing `backup`, `restore` and `needs_backup`, RoSys will automatically write the value to a file in the directory ~/.rosys/.
The filename contains the name of the module from where the `register` call was made.
After restarting the script, the value will be restored to its last state.

The `needs_backup` flag can be set to `True` to enforce a backup within RoSys' next backup cycle, which happens every 10 seconds.
During shutdown, all backups are performed, independent of the `needs_backup` flag.

The `backup` function can return any dictionary that represents the current state.
It should match the `restore` function so that it can translate it back to object state.

You should choose wisely which values to persist.
Try to avoid consuming unnecessary CPU and IO bandwidth for volatile things like wheel odometry or other sensor readings.

Note that the persistence module contains a number of helper functions:

- `to_dict`: converts (dictionaries or lists of) dataclasses into a dictionary (or list)
- `from_dict`: converts a dictionary into a dataclass of given type
- `replace_dict`: replaces the content of a dictionary using `from_dict` for each item
- `replace_list`: replaces the content of a list using `from_dict` for each item
- `replace_dataclass`: replaces the attributes of a dataclass with the values of a dictionary
