# Detector Masks

A detector can ignore parts of a camera image, like the robot's own chassis or the sky.
Its `masks` hold polygons per camera id in normalized image coordinates:
x and y run from 0 to 1 across the image, with the origin in the top-left corner.

The following example lets you draw such masks on the image of a simulated camera:

```python
{! examples/detector_masks/main.py !}
```

Each click adds a point to the outline, and pressing Enter or clicking the first point again turns the outline into a mask with `detector.masks.set`.
Clicking into a mask deletes it by setting the remaining polygons again.
The margin around the image lets you place points past its edge, so a mask is sure to reach the border.

The `DetectorSimulation` does not look at pixels, so it skips the simulated objects that are projected into a mask instead.
With a `DetectorHardware` the same masks gray out the pixels that are sent for detection and upload, while the image itself stays untouched.
