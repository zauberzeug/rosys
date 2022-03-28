import numpy as np
from rosys.actors import CameraProjector
from rosys.world import Camera


def test_projection():
    cam = Camera(id='1')
    cam.set_perfect_calibration(
        x=0.1, y=0.2, z=3,
        tilt_x=np.deg2rad(10), tilt_y=np.deg2rad(20),
        image_width=1600, image_height=1200,
    )
    CameraProjector.update_projection(cam, rows=3, columns=4)
    assert CameraProjector.allclose(cam.projection, [
        [[2.6401056200399315, -2.7431146775069135], [0.6428110258043753, -3.7739688197606194],
         [-3.507871324195061, -5.91624072397809], [-17.36746838820117, -13.069528518694367]],
        [[2.196224739529436, -0.1706616675668492], [0.38412315503644456, -0.2799447649973243],
         [-2.9432675693971544, -0.4806109792091923], [-11.05223001066152, -0.9696412521873483]],
        [[1.8490245289440204, 1.8414912905535379], [0.1961219592538731, 2.259334322339],
         [-2.5856242030237238, 2.962541680968452], [-8.25227030862712, 4.395033003953365]],
    ])

    cam = Camera(id='1')
    cam.set_perfect_calibration(tilt_x=np.deg2rad(70), image_width=1600, image_height=1200)
    CameraProjector.update_projection(cam, rows=3, columns=4,)
    assert CameraProjector.allclose(cam.projection, [
        [None, None, None, None],
        [[4.1035851536100045, -2.7474774194546225], [1.3678618050061626, -2.7474774194546216],
         [-1.3678614564615164, -2.7474774194546208], [-4.103585153610001, -2.74747741945462]],
        [[1.0543420148614626, -0.4354599872497414], [0.3514473606752155, -0.43545998724974133],
         [-0.35144727112297003, -0.4354599872497413], [-1.054342014861462, -0.4354599872497413]],
    ])


def test_allclose():
    assert CameraProjector.allclose([[[-1, 1], [1, 1]], [[-1, -1], [1, -1]]],
                                    [[[-1, 1], [1, 1]], [[-1, -1], [1, -1]]]) == True
    assert CameraProjector.allclose([[[-1, 1], [1, 1]], [[-1, -1], [1, -0]]],
                                    [[[-1, 1], [1, 1]], [[-1, -1], [1, -1]]]) == False
    assert CameraProjector.allclose([[[-1, 1], [1, 1]], [[-1, -1], None]],
                                    [[[-1, 1], [1, 1]], [[-1, -1], [1, -1]]]) == False
    assert CameraProjector.allclose([[[-1, 1], [1, 1]], [[-1, -1], None]],
                                    [[[-1, 1], [1, 1]], [[-1, -1], None]]) == True
