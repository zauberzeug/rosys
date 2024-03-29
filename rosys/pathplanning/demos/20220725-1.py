from rosys.geometry import Point, Pose
from rosys.pathplanning.area import Area
from rosys.pathplanning.planner_process import PlannerSearchCommand

cmd = PlannerSearchCommand(
    deadline=1658731937.3954191,
    areas=[
        Area(
            id='efc3d76a-dc4f-4578-9dba-b1b673dc63b2', type=None, color='green',
            outline=[Point(x=6.149937727671209, y=-0.17711484789623144),
                     Point(x=9.046821662232519, y=-0.21698541645168223),
                     Point(x=12.699061983842249, y=-0.2761903864398221),
                     Point(x=13.828295970670862, y=-0.6445640499531102),
                     Point(x=27.449221921371926, y=0.24219831882073173),
                     Point(x=28.024465321296177, y=-2.101078143446283),
                     Point(x=13.507960466210443, y=-3.1956976724428197),
                     Point(x=13.647888341643045, y=-5.411756660888658),
                     Point(x=12.238609367234528, y=-10.776575490209341),
                     Point(x=10.11621581471781, y=-10.751731291418032),
                     Point(x=10.152396176357877, y=-9.206962519468602),
                     Point(x=8.472012560929896, y=-8.997343306897463),
                     Point(x=6.073736496768359, y=-9.22290741066599),
                     Point(x=6.3645103740315045, y=-2.9410622155322494),
                     Point(x=-3.591146169525701, y=-1.6898632678909316),
                     Point(x=-4.920823257785388, y=-1.55630811794593),
                     Point(x=-5.308848849799811, y=0.8671983402731452),
                     Point(x=-2.9936880571586912, y=0.6627623167125547),
                     Point(x=-2.812650754483093, y=-0.0036416141580044026),
                     Point(x=0.881777645513987, y=-0.3116832474563821),
                     Point(x=1.0264402756201714, y=0.2055041290024313)]),
        Area(id='20c09f0d-eb4e-4bfd-a038-7613e7ee815c', type='sand', color='SandyBrown', outline=[]),
        Area(
            id='6e11cb8c-157e-4fc3-8f57-f436eaea5916', type='sand', color='SandyBrown',
            outline=[Point(x=-4.262672360611862, y=-0.07035923383821946),
                     Point(x=-4.1978165476055125, y=-1.537132009177664),
                     Point(x=5.222304593122139, y=-2.5157240058194037),
                     Point(x=5.482977498031892, y=-0.4112856025620827),
                     Point(x=1.4201278812513358, y=-0.11631869862536937),
                     Point(x=0.9875116671464726, y=-0.5521333871581944)]),
        Area(
            id='f9777f5e-8b58-4f24-94b6-6379aeca12b4', type='sand', color='SandyBrown',
            outline=[Point(x=26.981890753032665, y=-0.3263875500272597),
                     Point(x=15.807769756181198, y=-0.9329868641480514),
                     Point(x=15.003576366400255, y=-2.7753407211819487),
                     Point(x=26.2813145841394, y=-1.9524768591532027)])],
    obstacles=[],
    start=Pose(
        x=20.971612244244568, y=-0.8117194512230306, yaw=0.24142937993841057, time=1658731877.3894477),
    goal=Pose(x=9.37784533723036, y=-6.4125303042866575, yaw=3.108924746743267, time=0))

robot_outline = [(-0.22, -0.36), (1.07, -0.36), (1.17, 0), (1.07, 0.36), (-0.22, 0.36)]
