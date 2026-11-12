#!/usr/bin/env python3
from nicegui import ui
from nicegui.events import MouseEventArguments

import rosys
from rosys.geometry import Point, Point3d, Polygon
from rosys.vision import (
    DetectorSimulation,
    Image,
    SimulatedCalibratableCamera,
    SimulatedCameraProvider,
    SimulatedObject,
)

WIDTH, HEIGHT = 800, 600
MARGIN = 50  # space around the image for points past its edges

# a simulated camera and a detector with a few objects to detect
camera = SimulatedCalibratableCamera.create_calibrated(id='cam', width=WIDTH, height=HEIGHT)
camera_provider = SimulatedCameraProvider()
camera_provider.add_camera(camera)
detector = DetectorSimulation(camera_provider)
detector.simulated_objects = [SimulatedObject(category_name='object', position=Point3d(x=x, y=y, z=0))
                              for x in (-0.4, 0.0, 0.4) for y in (-0.3, 0.0, 0.3)]


# detection runs independently of the UI
async def detect() -> None:
    image = camera.latest_captured_image
    if image is not None:
        await detector.detect(image)


def masks() -> tuple[Polygon, ...]:
    return detector.masks.polygons.get(camera.id, ())


def svg_points(outline: list[Point]) -> str:
    return ' '.join(f'{point.x * WIDTH},{point.y * HEIGHT}' for point in outline)


@ui.page('/')
def page() -> None:
    outline: list[Point] = []

    # editing: the masks themselves live in the detector
    def finish_outline() -> None:
        if len(outline) >= 3:
            detector.masks.set(camera.id, [*masks(), Polygon(outline=outline.copy())])  # add a mask
            outline.clear()
            draw()

    def handle_click(e: MouseEventArguments) -> None:
        point = Point(x=(e.image_x - MARGIN) / WIDTH, y=(e.image_y - MARGIN) / HEIGHT)
        if outline and point.distance(outline[0]) < 0.03:
            finish_outline()
            return
        if not outline and any(mask.contains(point) for mask in masks()):
            detector.masks.set(camera.id, [mask for mask in masks() if not mask.contains(point)])  # delete a mask
        else:
            outline.append(point)
        draw()

    # drawing: masks and outline on the canvas, detections on the camera image
    def draw() -> None:
        canvas.set_content(''.join([
            f'<g transform="translate({MARGIN} {MARGIN})">',
            *(f'<polygon points="{svg_points(mask.outline)}" fill="gray" opacity="0.7" />' for mask in masks()),
            f'<polygon points="{svg_points(outline)}" fill="none" stroke="black" />',
            *(f'<circle cx="{point.x * WIDTH}" cy="{point.y * HEIGHT}" r="5" />' for point in outline[:1]),
            '</g>',
        ]))

    def update(image: Image) -> None:
        detections = image.get_detections(detector.name)
        camera_view.set_source(camera.get_image_url(image))
        camera_view.set_content(detections.to_svg() if detections else '')
        draw()

    # layout: the canvas is larger than the camera image and lies on top of it to catch the clicks
    ui.label('Click to outline a mask and finish it with Enter or a click on its first point. '
             'Click into a mask to delete it.')
    with ui.element().style('position: relative; outline: 1px solid gray'):
        camera_view = ui.interactive_image() \
            .style(f'position: absolute; left: {MARGIN}px; top: {MARGIN}px; width: {WIDTH}px')
        canvas = ui.interactive_image(size=(WIDTH + 2 * MARGIN, HEIGHT + 2 * MARGIN), on_mouse=handle_click, cross=True) \
            .style(f'width: {WIDTH + 2 * MARGIN}px')
    ui.keyboard(lambda e: e.key.enter and finish_outline())
    detector.NEW_DETECTIONS.subscribe(update)


rosys.on_repeat(detect, 0.1)

ui.run(title='RoSys')
