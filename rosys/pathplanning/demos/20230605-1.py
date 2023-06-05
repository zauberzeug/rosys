from rosys.geometry import Point, Pose
from rosys.pathplanning.area import Area
from rosys.pathplanning.obstacle import Obstacle
from rosys.pathplanning.planner_process import PlannerSearchCommand

robot_outline = [(-0.22, -0.36), (1.07, -0.36), (1.17, 0), (1.07, 0.36), (-0.22, 0.36)]

cmd = PlannerSearchCommand(
    deadline=1685967757.4002779,
    areas=[Area(id='37ae7616-47d0-41fe-82e7-2c888f91eb9e', type=None, color='green', outline=[]),
           Area(id='1f054f49-1d43-41f8-8724-86ce7d5cbe7f', type=None, color='green', outline=[]),
           Area(id='eb0eb71a-e49b-45b7-97ae-78731dca0ac1', type=None, color='green', outline=[]),
           Area(id='c1649868-485c-4cdb-a855-9566eabedb53', type=None, color='green', outline=[]),
           Area(id='53cc8ad2-386a-498b-ad81-54547495116f', type='sand', color='SandyBrown', outline=[]),
           Area(id='4625d140-d912-49a1-9057-c86b85340fb8', type=None, color='green', outline=[]),
           Area(id='ab56e934-05f0-4aee-bc65-94b5b0f7bf9d', type=None, color='green', outline=[]),
           Area(
               id='425af2ec-aba6-4fae-a7e7-e03323412532', type=None, color='green',
               outline=[Point(x=4.968311252375107, y=2.6421496575993833),
                        Point(x=5.1419204072123055, y=1.400933110746056),
                        Point(x=3.0795191107818938, y=1.3518823221383072),
                        Point(x=2.463648466961439, y=0.8979549953318877),
                        Point(x=-2.781434537374481, y=0.9966187403532025),
                        Point(x=-2.9350567299990793, y=-3.763374913739371),
                        Point(x=2.8562699579827924, y=-3.660320169010406),
                        Point(x=2.767054617434346, y=-8.488553113241306),
                        Point(x=-4.8324026088216785, y=-7.509903004312454),
                        Point(x=-4.344386408728945, y=5.228397676961204),
                        Point(x=-1.8440091361366333, y=5.148954345696822),
                        Point(x=0.5292150349675195, y=5.021872486750058),
                        Point(x=3.5133547720960143, y=4.724801634687276),
                        Point(x=3.3585051432016453, y=2.560636188622553)]),
           Area(
               id='ee758c16-3eab-4ac1-b0c0-f14478997113', type='sand', color='SandyBrown',
               outline=[Point(x=2.196837131237387, y=-8.039657380012908),
                        Point(x=2.449590494921049, y=-4.169000608686456),
                        Point(x=-4.357359286526278, y=-4.2031997339664375),
                        Point(x=-4.455773774638444, y=-7.108895619539001)]),
           Area(
               id='c103c09f-22d4-474c-90fa-5de369afda76', type='sand', color='SandyBrown',
               outline=[Point(x=-4.181057629305169, y=1.3128294229009871),
                        Point(x=-4.140939340451778, y=4.922764346384375),
                        Point(x=3.075903383561243, y=4.418375969310901),
                        Point(x=2.090385499720546, y=1.2057797418069605)])],
    obstacles=[
        Obstacle(
            id='5f21cbb5-493e-4777-b16d-faa0e1711c48',
            outline=[Point(x=-7.800655821889737, y=-14.788237374465119),
                     Point(x=-7.963014651933595, y=-14.62587854442126),
                     Point(x=-8.19262471135265, y=-14.62587854442126),
                     Point(x=-8.354983541396509, y=-14.788237374465119),
                     Point(x=-8.354983541396509, y=-15.017847433884175),
                     Point(x=-8.19262471135265, y=-15.180206263928033),
                     Point(x=-7.963014651933595, y=-15.180206263928033),
                     Point(x=-7.800655821889737, y=-15.017847433884175)])],
    start=Pose(x=-3.205682887784672, y=1.6995379059655211, yaw=5.869031314221408, time=1685967697.3665564),
    goal=Pose(x=-3.8376780529234553, y=1.3347154641633856, yaw=-1.625948318682161, time=0))
