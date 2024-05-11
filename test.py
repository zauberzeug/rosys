import multiprocessing
import traceback


def custom_set_start_method(method, force=False):
    # traceback.print_stack()
    multiprocessing.set_start_method(method, force)


multiprocessing.set_start_method = custom_set_start_method

if __name__ in ['__mp_main__', '__main__']:
    from nicegui import ui

    import rosys

    ui.run(reload=False)
