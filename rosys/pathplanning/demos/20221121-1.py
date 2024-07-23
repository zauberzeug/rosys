from rosys.geometry import Point, Pose
from rosys.pathplanning.area import Area
from rosys.pathplanning.planner_process import PlannerSearchCommand

robot_outline = [(-0.22, -0.36), (1.07, -0.36), (1.17, 0), (1.07, 0.36), (-0.22, 0.36)]

cmd = PlannerSearchCommand(
    deadline=1669033221.5299578,
    areas=[Area(id='884d3342-a97f-43b2-8ea8-fd86a28875a0', type=None, color='green', outline=[]),
           Area(id='eae34ef0-9408-4942-a885-b99ae320ee3e', type=None, color='green', outline=[]),
           Area(id='71ad69cb-a4d8-4bd1-962b-8914b570999e', type=None, color='green', outline=[]),
           Area(id='eccc3040-fb4e-4cbb-9962-8c9dedb5614d', type=None, color='green', outline=[]),
           Area(id='3d63962e-1844-4cea-86c9-7511d9a36538', type=None, color='green', outline=[]),
           Area(id='b3fce10e-8211-47a0-aaf0-54595cdcf6a9', type=None, color='green', outline=[]),
           Area(id='edadda55-fcf0-4fa4-8ea0-d6dede58ad20', type=None, color='green', outline=[]),
           Area(
               id='eeeb8091-669d-430e-a166-f3efe0cb577d', type=None, color='green',
               outline=[Point(x=-0.9820966992998833, y=0.5594010126687516),
                        Point(x=0.9818950057493285, y=0.4757498750285576),
                        Point(x=1.2009418122293585, y=-2.053035505252468),
                        Point(x=2.843348076287964, y=-2.1593415545043944),
                        Point(x=2.823523000408594, y=-6.622045769288673),
                        Point(x=23.39919876728811, y=-7.209585682853674),
                        Point(x=23.12961073329943, y=-4.727966640664446),
                        Point(x=25.44211500436315, y=-4.917117601537843),
                        Point(x=25.254057459897428, y=-7.30039547190418),
                        Point(x=31.117322871310584, y=-7.30874976814529),
                        Point(x=32.964355462671385, y=-14.055695954096095),
                        Point(x=24.649416414608936, y=-11.109546248629433),
                        Point(x=17.616584250042685, y=-10.884063308496602),
                        Point(x=9.223666135228097, y=-17.38133296632852),
                        Point(x=5.689716692550434, y=-17.369675746326187),
                        Point(x=5.933913041471273, y=-12.289056302853467),
                        Point(x=10.043294238862536, y=-12.429876990525997),
                        Point(x=11.034740817683726, y=-10.57811622159867),
                        Point(x=-4.2009326571530945, y=-9.69982650057301),
                        Point(x=-5.039999866742331, y=0.964844412616598),
                        Point(x=-2.9058855259532823, y=2.025225894721393),
                        Point(x=-2.0701300010795, y=-1.5329865054838625),
                        Point(x=-0.8484091464298582, y=-1.5649771708990738)]),
           Area(id='b167b654-0a92-4105-a7c1-99f2d7d77b8e', type=None, color='green', outline=[]),
           Area(id='5a1c0801-effb-45dc-a767-9d94f0ecde3f', type=None, color='green', outline=[]),
           Area(id='5955a881-213a-4caf-9a44-b040e88f8bc0', type=None, color='green', outline=[]),
           Area(id='2f2359a4-10ff-4c87-8a11-26104c4a98d8', type=None, color='green', outline=[]),
           Area(
               id='32a8b259-f6f3-4ac5-8647-29c1d6199d7e', type='sand', color='SandyBrown',
               outline=[Point(x=2.4483024578850547, y=-2.6044213457061494),
                        Point(x=-4.383158752622126, y=-2.8505324325536794),
                        Point(x=-3.6846084160555117, y=-9.376194464356086),
                        Point(x=24.81256179170486, y=-10.727967018383495),
                        Point(x=32.55574304114366, y=-13.491486506182707),
                        Point(x=30.907911264919846, y=-7.777305369490148),
                        Point(x=2.6010099251238747, y=-6.900331769639822)]),
           Area(id='a76d45b8-5427-4afe-989b-7e8ea0ba277e', type=None, color='green', outline=[])],
    obstacles=[],
    start=Pose(
        x=24.270122886242437, y=-6.361720240897067, yaw=-23.522200946848816, time=1669033201.5081003),
    goal=Pose(x=0.6666251770143659, y=-8.458059112178587, yaw=0.8726646259971644, time=0))
