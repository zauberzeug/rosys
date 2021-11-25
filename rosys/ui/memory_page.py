from nicegui.elements.log import Log
from nicegui.ui import Ui
import rosys
import rosys.ui
from outside_in_tracking import World
from rosys import event
from datetime import datetime
import objgraph


class MemoryPage:
    ui: Ui = None  # will be set by rosys.ui.configure

    def __init__(self) -> None:
        with self.ui.page('/memory'):
            self.ui.label('Analyze Memory')
            self.ui.button('log most common types', on_click=lambda: objgraph.show_most_common_types(limit=20))
            self.ui.button('log growth', on_click=lambda: objgraph.show_growth(limit=20))
