#!/usr/bin/env python3
from nicegui import ui
import numpy as np
import pylab as pl
import time
from binary_renderer import BinaryRenderer

renderer = BinaryRenderer((30, 40))

x, y, r = 10, 15, 5
t = time.time()
renderer.circle(x, y, r)
ui.label(f'circle:  {(time.time() - t) * 1000:.3f} ms')

points = np.array([[20.5, 10], [30, 15], [25, 25]])
t = time.time()
renderer.polygon(points)
ui.label(f'polygon:  {(time.time() - t) * 1000:.3f} ms')

with ui.plot():
    pl.imshow(renderer.map, cmap=pl.cm.gray, interpolation='nearest')
    pl.gca().add_patch(pl.matplotlib.patches.Circle((x, y), r, color='none', ec='C0'))
    pl.fill(points[:, 0], points[:, 1], color='none', ec='C0')

ui.run()
