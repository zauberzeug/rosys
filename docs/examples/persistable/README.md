# Persistable

The `Persistable` class is a mixin for objects that need to be persisted.
It provides a `persistent` method that can be used to make the object persistent.

Derived classes must implement the `backup_to_dict` and `restore_from_dict` methods.

```python
{! examples/persistable/main.py !}
```

## Export and import

The `export_all` method can be used to export all persistable objects to a dictionary.
The `import_all` method can be used to import all persistable objects from a dictionary.
Likewise, `export_button` and `import_button` are UI elements based on these two methods that can be added to a page.

## Migration from the old `PersistentModule` class

The `PersistentModule` class has been replaced by the `Persistable` mixin.
To migrate, follow these steps:

1. Replace `PersistentModule` with `Persistable` in the class definition.
2. Rename the `backup` and `restore` methods to `backup_to_dict` and `restore_from_dict`.
3. Use the `persistent` method to make objects persistent.
4. If you called `PersistentModule` with a `persistence_key`,
   remove it and use the `key` parameter of the `persistent` method instead.
