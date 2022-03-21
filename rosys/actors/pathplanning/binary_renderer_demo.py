#!/usr/bin/env python3
from nicegui import ui
import numpy as np
import pylab as pl
import time
from rosys.actors.pathplanning import BinaryRenderer

renderer = BinaryRenderer((30, 40), fill_value=True)

area_outline = np.array([[-5, 5], [35, 3], [40, 25], [5, 30]])
t = time.time()
renderer.polygon(area_outline, False)
ui.label(f'area: {(time.time() - t) * 1000:.3f} ms')

x, y, r = 10, 15, 5
t = time.time()
renderer.circle(x, y, r)
ui.label(f'circle: {(time.time() - t) * 1000:.3f} ms')

polygon_outline = np.array([[20.5, 10], [30, 15], [25, 25]])
t = time.time()
renderer.polygon(polygon_outline)
ui.label(f'polygon: {(time.time() - t) * 1000:.3f} ms')

with ui.plot():
    pl.imshow(renderer.map, cmap=pl.cm.gray, interpolation='nearest')
    pl.xlim(0, renderer.map.shape[1] - 1)
    pl.ylim(renderer.map.shape[0] - 1, 0)
    pl.gca().add_patch(pl.matplotlib.patches.Circle((x, y), r, color='none', ec='C0', lw=2))
    pl.fill(polygon_outline[:, 0], polygon_outline[:, 1], color='none', ec='C0', lw=2)
    pl.fill(area_outline[:, 0], area_outline[:, 1], color='none', ec='C2', lw=2)

ui.run()
