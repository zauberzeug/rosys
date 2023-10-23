import numpy as np

from rosys.vision import CalibratedCameraMixin, Camera, CameraProjector


class CalibratedCamera(CalibratedCameraMixin, Camera):
    def __init__(self, **kwargs) -> None:
        CalibratedCameraMixin.__init__(self)
        Camera.__init__(self, **kwargs)


def test_projection():
    cam = CalibratedCamera(id='1')
    cam.set_perfect_calibration(
        x=0.1, y=0.2, z=3, roll=np.deg2rad(180 + 10), pitch=np.deg2rad(20), width=1600, height=1200)
    coordinates = CameraProjector.project(cam, rows=3, columns=4)
    assert CameraProjector.allclose(coordinates, [
        [[-17.36746838820115, 13.46952851869435], [-3.5078721380854008, 6.316241144047431],
         [0.6428106822103088, 4.173968997098186], [2.6401056200399307, 3.143114677506913]],
        [[-11.05223001066152, 1.369641252187346], [-2.9432681705678023, 0.880611015464217],
         [0.38412285609758134, 0.6799447830255427], [2.1962247395294354, 0.5706616675668484]],
        [[-8.252270308627121, -3.9950330039533677], [-2.5856246784528123, -2.5625418011538383],
         [0.19612169506058386, -1.8593343891253478], [1.8490245289440204, -1.4414912905535386]],
    ])

    cam = CalibratedCamera(id='1')
    cam.set_perfect_calibration(roll=np.deg2rad(180+70), width=1600, height=1200)
    coordinates = CameraProjector.project(cam, rows=3, columns=4,)
    assert CameraProjector.allclose(coordinates, [
        [None, None, None, None],
        [[-4.103585153610006, 2.747477419454624], [-1.3678618050061633, 2.747477419454624],
         [1.367861456461518, 2.747477419454624], [4.103585153610006, 2.747477419454624]],
        [[-1.0543420148614624, 0.43545998724974166], [-0.3514473606752154, 0.43545998724974166],
         [0.35144727112297025, 0.43545998724974166], [1.0543420148614624, 0.43545998724974166]],
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
