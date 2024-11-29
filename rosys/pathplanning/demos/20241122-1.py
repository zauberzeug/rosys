from rosys.geometry import Point, Pose
from rosys.pathplanning.area import Area
from rosys.pathplanning.obstacle import Obstacle
from rosys.pathplanning.planner_process import PlannerSearchCommand

robot_outline = [(-0.22, -0.36), (1.07, -0.36), (1.17, 0), (1.07, 0.36), (-0.22, 0.36)]

cmd = PlannerSearchCommand(deadline=1732182006.1802719, areas=[Area(outline=[Point(x=-0.7335416821810565, y=1.8819432681699386), Point(x=-0.7250165161960502, y=1.3156265458758731), Point(x=-0.7191624231236626, y=0.7050143829825803), Point(x=-0.9392354907231761, y=0.6600732307877654), Point(x=-1.554409002112933, y=0.6903064898737243), Point(x=-2.178309427247247, y=0.7230733690694867), Point(x=-2.6389117342870763, y=1.471430235358901), Point(x=-3.3074133973680833, y=1.5947369479900462), Point(x=-4.189191814653708, y=1.7967673138751714), Point(x=-4.149243088237694, y=1.2417047684095357), Point(x=-4.140803984852408, y=0.6983329995466369), Point(x=-4.181079051861245, y=0.15457704941950795), Point(x=-4.267426718470572, y=-0.41009037231880496), Point(x=-4.405141540730216, y=-1.0169624156745454), Point(x=-4.613335711010887, y=-1.6960157288922628), Point(x=-4.913227417445498, y=-2.490071806102605), Point(x=-5.236098908663235, y=-3.395101823135409), Point(x=-4.615314835540341, y=-3.327147004931693), Point(x=-3.994451362595439, y=-3.2239259533923907), Point(x=-3.4547484101363364, y=-3.150966016598824), Point(x=-2.9861593089493086, y=-3.114381903446658), Point(x=-2.1706803567998407, y=-3.110441665209578), Point(x=-1.4339221242679008, y=-3.153548900293729), Point(x=-1.2252432460986546, y=-2.3717082875866358), Point(x=-1.049282004681811, y=-1.6885735677918512), Point(x=-0.7647603183709843, y=-1.716785027530492), Point(x=-0.7431510886816872, y=-2.3401439028094884), Point(x=-0.7265859700826607, y=-3.026054511639549), Point(x=0.035557315265264317, y=-2.953956990344677), Point(x=0.7703997262819586, y=-2.898252353665584), Point(x=1.5006887505414563, y=-2.8754586227000187), Point(x=2.2924001873263795, y=-2.9308171661739673), Point(x=2.1533955837738863, y=-2.34408888976562), Point(x=2.0617185044561395, y=-1.8434899532080946), Point(x=1.9427584683116634, y=-0.9674639420286982), Point(x=1.6270762475427993, y=-0.4993797056435728), Point(x=1.3293892146504056, y=-0.04581669093897631), Point(x=0.8740369473853011, y=0.31427132423941406), Point(x=0.41183503551527706, y=0.6732642167042276), Point(x=0.3403672631309441, y=1.2879595496674496), Point(x=0.27858673563611375, y=1.8597777981594785), Point(x=-0.22000641250396719, y=1.8709606898274715)], id='e15e8e3f-5eff-4c75-87b3-805182a21d06', type='path', color='DodgerBlue', closed=True), Area(outline=[Point(x=-7.002183230175394, y=1.7821339308732753), Point(x=-6.230899031041709, y=1.8272399141938624), Point(x=-5.532944157114525, y=1.8808023446293314), Point(x=-4.876431617064556, y=1.934999101297615), Point(x=-4.250871577892205, y=1.9835540770992064), Point(x=-3.6580404811245133, y=2.0225507547136843), Point(x=-3.1035453650534968, y=2.0521091905387587), Point(x=-2.106075746569589, y=2.1018715677082045), Point(x=-1.9884359805678715, y=1.1487632733481075), Point(x=-1.2924621987815739, y=0.9344527466795811), Point(x=-1.3210731336455448, y=0.1616557085015004), Point(x=-1.3420327574409763, y=-0.6267185285636968), Point(x=-1.3330784357294774, y=-1.4580457262171262), Point(x=-1.2269938520550423, y=-2.4464908803019054), Point(x=-1.2645063512045316, y=-3.3133145335079095), Point(x=-1.8838655745793775, y=-3.2605522950966916), Point(x=-2.4822421916835373, y=-3.23788662274237), Point(x=-3.0691045901375147, y=-3.272493473368841), Point(x=-3.6717216457014725, y=-3.3514381719281827), Point(x=-4.406472731356447, y=-2.681078576813847), Point(x=-5.18013973612608, y=-2.0810643822107306), Point(x=-5.5890582577495795, y=-1.775752093110361), Point(x=-6.0135370996848, y=-1.4607762885873674), Point(x=-6.455241463167528, y=-1.1351168106250205), Point(x=-6.9193256664889145, y=-0.798992058538447), Point(x=-6.932413830280107, y=-0.14688451011447456), Point(x=-6.951084133904734, y=0.49851645230623), Point(x=-6.972969624420697, y=1.140337675949381)], id='ac80a592-89a6-4126-8b2b-3f051174cb9d', type='path', color='DodgerBlue', closed=True), Area(outline=[Point(x=-3.648543854355812, y=-3.2001797907524687), Point(x=-4.3625730679970545, y=-2.5737194723280368), Point(x=-5.115864082177901, y=-1.997440943581617), Point(x=-5.5133122903394, y=-1.702021972231361), Point(x=-5.925008083239463, y=-1.3966512445310846), Point(x=-6.352042905909946, y=-1.0806340552637157), Point(x=-6.79844726374362, y=-0.7543885454256544), Point(x=-6.811822642691272, y=-0.12874097322050226), Point(x=-6.828687262659409, y=0.49153217254250614), Point(x=-6.846895238417157, y=1.1083146850194023), Point(x=-6.869672313886208, y=1.7237121915285525), Point(x=-6.2369959557006505, y=1.7598860232770495), Point(x=-5.653929995896809, y=1.8016704608441692), Point(x=-5.101111599331976, y=1.8447506687402275), Point(x=-4.570111139457897, y=1.8854272382612747), Point(x=-3.9073587279046693, y=1.9243885625718824), Point(x=-3.2866966664700694, y=1.9521369071211954), Point(x=-2.7136227383207254, y=1.9717970418808237), Point(x=-2.183154593140967, y=1.9904781874570596), Point(x=-2.0892523999455834, y=1.1076884224216748), Point(x=-1.3495043404712947, y=0.8550794524019857), Point(x=-1.3842151801610025, y=0.09941559066534122), Point(x=-1.413537735052312, y=-0.6714009198078071), Point(x=-1.4170397142316877, y=-1.4819730707906542), Point(x=-1.3365167643495417, y=-2.435031030394372), Point(x=-1.359491413436869, y=-3.205452860376947), Point(x=-2.1772170914812587, y=-3.123818340162155), Point(x=-2.9058256189831173, y=-3.1277230245613454)], id='7b7a542c-6096-48c3-8e62-be055153cc1b', type='sand', color='SandyBrown', closed=True), Area(outline=[Point(x=-5.409825755717819, y=-1.3486896714593035), Point(x=-4.418654935396619, y=-1.3971795426671285), Point(x=-3.9232984971275426, y=-1.3827306181608483), Point(x=-3.4183279711394072, y=-1.3417805955887356), Point(x=-2.895874510149592, y=-1.273045573006959), Point(x=-2.349257649265044, y=-1.1799213939814708), Point(x=-1.7806358179914006, y=-1.0820912252186634), Point(x=-1.2101615610273124, y=-1.0178906504652456), Point(x=-1.2691469901607912, y=-1.8394555456730304), Point(x=-1.3635938135757018, y=-2.6702847076507132), Point(x=-1.389902213163063, y=-3.373172048043753), Point(x=-1.3711278538965255, y=-4.014386795295228), Point(x=-1.3269138708139145, y=-4.636647796262337), Point(x=-1.2686305632944341, y=-5.261297460631184), Point(x=-1.2037300639082686, y=-5.898677994397625), Point(x=-1.1369446321766468, y=-6.552602743230552), Point(x=-2.0467596493356814, y=-6.516932914926213), Point(x=-2.954448312941236, y=-6.479724351221383), Point(x=-3.8800321908826776, y=-6.442344682078464), Point(x=-4.808464906329679, y=-6.406963981620004), Point(x=-4.818934305837635, y=-7.070284665583645), Point(x=-4.8267080671038265, y=-7.74257738230307), Point(x=-5.657793090413397, y=-7.689754922735526), Point(x=-6.456422224075708, y=-7.634306495039068), Point(x=-7.233148295502026, y=-7.5828501611471255), Point(x=-8.020795112231752, y=-7.541187665596438), Point(x=-7.970797391532123, y=-6.564831921991668), Point(x=-7.931273778537458, y=-5.622434232546228), Point(x=-6.968247443970489, y=-5.624221443959908), Point(x=-6.474113257052833, y=-5.571828691638769), Point(x=-5.973561762165933, y=-5.5169573975087856), Point(x=-5.540871531484371, y=-5.024338902277403), Point(x=-5.112090295320412, y=-4.545713159635389), Point(x=-4.9487376012131215, y=-4.015327355626931), Point(x=-4.792159302987202, y=-3.4899292026880726), Point(x=-4.811458579500462, y=-2.8623648296484037), Point(x=-4.838919957284041, y=-2.157941637959813)], id='3c8534b0-758f-4442-a2b7-c9f52438fdd1', type='path', color='DodgerBlue', closed=True), Area(outline=[Point(x=-7.448251754748485, y=-5.815743706861782), Point(x=-7.48254532077896, y=-6.6095639828333645), Point(x=-7.515583283298628, y=-7.428271244027151), Point(x=-6.961061865918973, y=-7.458046580766507), Point(x=-6.404038658518599, y=-7.491283144384499), Point(x=-5.834241334389822, y=-7.526143852261059), Point(x=-5.247051131004228, y=-7.560490055189995), Point(x=-5.129229684672656, y=-6.912431080070599), Point(x=-5.008896297820032, y=-6.273032925352798), Point(x=-4.057922514800175, y=-6.316054457985844), Point(x=-3.1032167459880196, y=-6.363161277865452), Point(x=-2.1664107932402428, y=-6.411030185367062), Point(x=-1.2315717030726194, y=-6.456851671037912), Point(x=-1.3191134739584305, y=-5.571883262230861), Point(x=-1.400147410468843, y=-4.715704339643839), Point(x=-1.4580655306579144, y=-3.8678306090364676), Point(x=-1.4632055611678263, y=-2.9704742386980265), Point(x=-1.3809025401268973, y=-2.1091271239175162), Point(x=-1.2711918578349035, y=-1.1489153225243065), Point(x=-1.8148883956477722, y=-1.233885228901646), Point(x=-2.3471111411477854, y=-1.3350722862682183), Point(x=-2.8545983135605697, y=-1.4218790663732035), Point(x=-3.339289535114897, y=-1.4841368166407252), Point(x=-4.267084817368692, y=-1.538297067203847), Point(x=-5.179983307675796, y=-1.507811279175236), Point(x=-4.707569134675155, y=-2.182522339941535), Point(x=-4.692392879752872, y=-2.7704829598551988), Point(x=-4.6815313034530295, y=-3.2993933378521434), Point(x=-4.779836720782619, y=-3.9696388409655), Point(x=-4.8849202532142595, y=-4.645209649733215), Point(x=-5.401943316893433, y=-5.163913159981487), Point(x=-5.923175202247121, y=-5.699447562236087), Point(x=-6.690210157332352, y=-5.760798882801558)], id='64f5a5b0-ed15-4b49-839a-f74d0e0d746b', type='sand', color='SandyBrown', closed=True), Area(outline=[Point(x=-10.690619552813867, y=-17.198702769616364), Point(x=-10.047797472766701, y=-16.873523930763266), Point(x=-9.504232607332746, y=-16.58526250223942), Point(x=-8.822812713717532, y=-16.70523028200997), Point(x=-8.205621442111863, y=-16.774983298320784), Point(x=-7.618300997445589, y=-16.815864909382356), Point(x=-7.042206084252218, y=-16.841332406727716), Point(x=-6.46799638673372, y=-16.859695308403104), Point(x=-5.891585335812572, y=-16.876679252363576), Point(x=-5.312311606755463, y=-16.895986465620258), Point(x=-4.731421482111399, y=-16.91905723698988), Point(x=-3.921636538831873, y=-16.99603892133779), Point(x=-3.093991172603264, y=-17.072131571669445), Point(x=-2.2326995684312667, y=-17.132697207161762), Point(x=-1.2929347654634356, y=-17.152561638968816), Point(x=-1.5252327615088728, y=-17.755856671150575), Point(x=-1.7425168783916831, y=-18.45113489503122), Point(x=-1.9886919918808874, y=-19.33849877173764), Point(x=-2.137814609209759, y=-19.89637460209725), Point(x=-2.3155805916113557, y=-20.562990378289772), Point(x=-2.833613079402574, y=-20.536067093012292), Point(x=-3.356823673748788, y=-20.50898030790722), Point(x=-3.885423599304376, y=-20.48181543414902), Point(x=-4.418632458476359, y=-20.45470306734876), Point(x=-4.954753060148664, y=-20.427798709281014), Point(x=-5.491388540118579, y=-20.401267612369907), Point(x=-6.0258173821033, y=-20.3752521840999), Point(x=-6.555448837263529, y=-20.349850137134883), Point(x=-7.078243957081773, y=-20.325101623685924), Point(x=-7.59300139751824, y=-20.300980168665152), Point(x=-8.099488621344975, y=-20.277403842083373), Point(x=-8.598429888139954, y=-20.254239787595733), Point(x=-9.580951459520396, y=-20.20842490355504), Point(x=-10.563497374932465, y=-20.161716822490018), Point(x=-10.517860012304261, y=-19.2114286969799), Point(x=-10.515387796470394, y=-18.450574063750285), Point(x=-10.567094238973485, y=-17.80059444758589)], id='176e0d84-1bc3-4b2c-92b7-71394351588a', type='path', color='DodgerBlue', closed=True), Area(outline=[Point(x=-6.903424213846128, y=-7.553693298823358), Point(x=-6.989200794558625, y=-7.026680434258032), Point(x=-7.069339511231144, y=-6.518146049127873), Point(x=-7.198515181862743, y=-5.533723794813491), Point(x=-7.8176172801671635, y=-5.523616088079704), Point(x=-8.461874389273653, y=-5.48880100173311), Point(x=-9.160495699166917, y=-5.4418918349958885), Point(x=-9.930052760638022, y=-5.391968803740896), Point(x=-10.076129512378376, y=-4.827215454382149), Point(x=-10.218653870356462, y=-4.253863606488494), Point(x=-10.310429066668535, y=-3.666830737461281), Point(x=-10.412842403526053, y=-2.9649356907267057), Point(x=-11.066419676652028, y=-2.8790928983387882), Point(x=-11.789059704310692, y=-2.7460328540796484), Point(x=-12.611955502608122, y=-2.5504452688975716), Point(x=-13.073888494971929, y=-2.4242833717013177), Point(x=-13.575287368966112, y=-2.2807011284435568), Point(x=-13.4657787673157, y=-3.0858920940690133), Point(x=-13.4177126142321, y=-3.7782057170902315), Point(x=-13.427788251177812, y=-4.394115960719182), Point(x=-13.474959942264794, y=-4.982702755870555), Point(x=-13.546834989031026, y=-5.572208816903998), Point(x=-13.635648182190373, y=-6.17973265426845), Point(x=-13.735674314344463, y=-6.816165876846591), Point(x=-13.842430689435634, y=-7.488118409952021), Point(x=-13.952876343546484, y=-8.199054584355693), Point(x=-14.066117914427775, y=-8.950765884607723), Point(x=-14.184227654173249, y=-9.745661705097083), Point(x=-14.313008517721943, y=-10.589921674905018), Point(x=-14.463042641109055, y=-11.497705649603457), Point(x=-14.551179775678865, y=-11.983563777679517), Point(x=-14.652129334332356, y=-12.497852863981244), Point(x=-14.770292563671367, y=-13.048729544348367), Point(x=-14.911846967513853, y=-13.647995496486676), Point(x=-15.085754170227204, y=-14.31315558541079), Point(x=-15.305265289156, y=-15.07069150035068), Point(x=-14.405302271823885, y=-14.902858636807967), Point(x=-13.600728118327895, y=-14.781758870910043), Point(x=-12.866846472951735, y=-14.693641495444611), Point(x=-12.187683726027325, y=-14.630137863861492), Point(x=-11.552352117653278, y=-14.586527593307686), Point(x=-10.952845080095, y=-14.560773310979064), Point(x=-10.382610572398416, y=-14.553049380360065), Point(x=-9.835511775025415, y=-14.56563183089845), Point(x=-9.83215774487531, y=-13.678344531339123), Point(x=-9.827640298437922, y=-12.931373697630482), Point(x=-9.822393901526812, y=-12.268748036453644), Point(x=-9.816666435903837, y=-11.659957386439117), Point(x=-9.810626740616094, y=-11.086790470456684), Point(x=-9.804419620834594, y=-10.537723657721088), Point(x=-9.798194442881774, y=-10.005440863740104), Point(x=-9.79211510865346, y=-9.485730071574913), Point(x=-9.136700909824892, y=-9.45352684356441), Point(x=-8.535607469784571, y=-9.419543184012602), Point(x=-8.597248981743785, y=-8.546828889705004), Point(x=-8.66401616286079, y=-7.713560418730837), Point(x=-7.728090329357373, y=-7.628906709947848)], id='ac434d6c-08ec-4503-87de-09a4563e8731', type='path', color='DodgerBlue', closed=True), Area(outline=[Point(x=-9.011852845389514, y=-5.558504359386527), Point(x=-10.006165006976204, y=-5.477425470512236), Point(x=-10.151358109296384, y=-4.925501668097747), Point(x=-10.293134188324547, y=-4.370167653413143), Point(x=-10.43721920138827, y=-3.777964784940066), Point(x=-10.595108838896746, y=-3.080978683202252), Point(x=-11.17977242392603, y=-3.0120773395356863), Point(x=-11.819522418789601, y=-2.909759560686143), Point(x=-12.535763423227973, y=-2.7629200194623955), Point(x=-13.359955187740447, y=-2.5564241346099994), Point(x=-13.27244351582249, y=-3.3021195843557756), Point(x=-13.248566981047993, y=-3.9387065624722193), Point(x=-13.270659935016472, y=-4.519176189061145), Point(x=-13.322538290561402, y=-5.082414145871123), Point(x=-13.394799362817745, y=-5.650989079253679), Point(x=-13.481171838189237, y=-6.239092356292854), Point(x=-13.5767644825223, y=-6.85591816260383), Point(x=-13.677602962503776, y=-7.5069758718669295), Point(x=-13.780918168485687, y=-8.194948391858352), Point(x=-13.885808231353217, y=-8.920955842108171), Point(x=-13.993946278354215, y=-9.686571725294923), Point(x=-14.110177880770667, y=-10.49654379740803), Point(x=-14.243244949613505, y=-11.362272953906592), Point(x=-14.407445745961382, y=-12.307003483445062), Point(x=-14.508185180743563, y=-12.821794736222092), Point(x=-14.627073005451681, y=-13.376071210064417), Point(x=-14.770618629255914, y=-13.982981120522119), Point(x=-14.94830975616192, y=-14.662064142502588), Point(x=-14.155449701838037, y=-14.555169442255917), Point(x=-13.433599715079819, y=-14.479011416181951), Point(x=-12.766431771576471, y=-14.425130800951266), Point(x=-12.14298780034084, y=-14.388313558717186), Point(x=-11.555538385432246, y=-14.365642759840357), Point(x=-10.998227809686103, y=-14.355982226925818), Point(x=-10.466159476999769, y=-14.359720284706263), Point(x=-9.954703038983629, y=-14.378727447609412), Point(x=-9.945953505148976, y=-13.513551468827764), Point(x=-9.937789570725016, y=-12.776148839409213), Point(x=-9.929969672441668, y=-12.116301408786468), Point(x=-9.922364400608302, y=-11.506188759345221), Point(x=-9.914909120824419, y=-10.928994521950981), Point(x=-9.907581943607946, y=-10.374071769658263), Point(x=-9.900400054524384, y=-9.834819650208274), Point(x=-9.893412837355303, y=-9.307767439915304), Point(x=-9.05164803271064, y=-9.299403285943706), Point(x=-9.038273510456506, y=-8.75986716772947), Point(x=-9.027612095723036, y=-8.236492928581344), Point(x=-9.01585525068013, y=-7.25079992657728), Point(x=-9.013444854639928, y=-6.361418860582584)], id='99ed873c-e9c3-4217-9ac5-211ecbd06464', type='sand', color='SandyBrown', closed=True), Area(outline=[Point(x=-8.570834280906874, y=-7.641169067484498), Point(x=-7.7600877129389385, y=-7.586953526667642), Point(x=-7.0352889919037285, y=-7.53748875011623), Point(x=-7.102430968274792, y=-7.026142150441628), Point(x=-7.165327208244728, y=-6.533741307904595), Point(x=-7.264779929618003, y=-5.585282017012167), Point(x=-7.9388411269171595, y=-5.604276015581691), Point(x=-8.653528560865528, y=-5.60004412400728), Point(x=-8.612767940477825, y=-6.562980438345009), Point(x=-8.590711391784122, y=-7.086184138948853)], id='0fcb59a8-028b-43ea-9cfc-c77441660060', type='sand', color='SandyBrown', closed=True), Area(outline=[Point(x=-5.382226769659346, y=-0.9025849007302444), Point(x=-5.431915680965371, y=-0.15912894930242139), Point(x=-5.453511167161683, y=0.5573065187288531), Point(x=-5.423791400853386, y=1.2791913082423265), Point(x=-5.305647655726064, y=2.066698944851106), Point(x=-6.259107394329053, y=2.0775053153780476), Point(x=-7.084033943144275, y=2.168681504425303), Point(x=-7.593653246038695, y=1.6227428888242703), Point(x=-8.355645814058146, y=1.6358665179452463), Point(x=-8.924725358777149, y=2.2383650827816073), Point(x=-9.695430432067187, y=2.216077729166161), Point(x=-10.491784429061674, y=2.2030276749174567), Point(x=-11.32785260065203, y=2.20883466721143), Point(x=-12.243889780668637, y=2.2607757649549045), Point(x=-12.302218621051754, y=1.6104819611556382), Point(x=-12.399948665076785, y=0.9780883587273077), Point(x=-12.522460196796967, y=0.33126039144876795), Point(x=-12.661264864193754, y=-0.34697505364183123), Point(x=-12.81150672716437, y=-1.0655918117350596), Point(x=-12.972484544512913, y=-1.8298655576898377), Point(x=-13.149387522263796, y=-2.6464682516787628), Point(x=-13.355957499934387, y=-3.530790646314205), Point(x=-13.61228135064152, y=-4.491256174342574), Point(x=-13.776149424399978, y=-5.030010684423786), Point(x=-13.978332911541392, y=-5.6294668379655946), Point(x=-13.35698670597559, y=-5.3906142323999795), Point(x=-12.804638115844378, y=-5.199091273668316), Point(x=-12.304356681291232, y=-5.498911011736056), Point(x=-11.807817534120513, y=-5.808914257523148), Point(x=-11.311706680604392, y=-6.130312798091222), Point(x=-10.813625587867383, y=-6.466446212185183), Point(x=-10.655974413039937, y=-5.74043726111634), Point(x=-10.519608735835225, y=-5.095946959412502), Point(x=-10.396870952548571, y=-4.503527893941975), Point(x=-10.28313112512496, y=-3.945628283787622), Point(x=-10.17536824375528, y=-3.411244675485102), Point(x=-10.07153688571541, y=-2.8935681826818755), Point(x=-9.970292011970342, y=-2.388950401008258), Point(x=-9.870889782833013, y=-1.8964149508011554), Point(x=-9.19104869226697, y=-1.9497388875689168), Point(x=-8.523647396320154, y=-1.9972205850671738), Point(x=-8.51506198523767, y=-1.3445574535555023), Point(x=-8.510762996237887, y=-0.7221671072555048), Point(x=-7.661426490068443, y=-0.7316831118567269), Point(x=-6.857862896061018, y=-0.7410289814369999), Point(x=-6.134368231391243, y=-0.8247472584932993)], id='50d416f4-24d8-4dfd-b7f4-d3ed093cace7', type='path', color='DodgerBlue', closed=True), Area(outline=[Point(x=-5.429783476536418, y=2.0490681931953594), Point(x=-6.277803549413565, y=2.0205315530125794), Point(x=-7.018943430021978, y=2.0634559602788363), Point(x=-7.559302510639149, y=1.5666500827243062), Point(x=-8.380716970680776, y=1.5644475381815246), Point(x=-8.917820494099999, y=2.151917317131423), Point(x=-9.67645693907531, y=2.116635485815798), Point(x=-10.459864634934823, y=2.087276842440902), Point(x=-11.278741566132895, y=2.07107721939317), Point(x=-12.164655086448073, y=2.0890313249452968), Point(x=-12.25342969491308, y=1.341005584877841), Point(x=-12.38482476249772, y=0.5935707782037374), Point(x=-12.54158840397436, y=-0.1906930646812363), Point(x=-12.714089632457474, y=-1.0293607274766923), Point(x=-12.899933388702289, y=-1.9309373943778305), Point(x=-13.107574534237475, y=-2.9059586644871573), Point(x=-13.226542253140742, y=-3.4288153678735034), Point(x=-13.362391580962107, y=-3.9839707263961883), Point(x=-13.523563556375947, y=-4.583414329590431), Point(x=-13.72352519184275, y=-5.2473075302253), Point(x=-13.19924309666052, y=-5.135775692676811), Point(x=-12.715283485314313, y=-5.048215381797774), Point(x=-12.255120553250176, y=-5.3430291301492225), Point(x=-11.796772532722432, y=-5.646214195613717), Point(x=-11.337886835217164, y=-5.958824012592342), Point(x=-10.876771653298741, y=-6.283519058195143), Point(x=-10.714916890180257, y=-5.598147871903785), Point(x=-10.57297798068422, y=-4.983506162565964), Point(x=-10.44403943965497, y=-4.414960233651044), Point(x=-10.323811439400961, y=-3.877459327486334), Point(x=-10.209448868990343, y=-3.361416788436207), Point(x=-10.099015326663185, y=-2.8608538596088486), Point(x=-9.885441406339316, y=-1.895702240506102), Point(x=-9.232959363049215, y=-1.925552952846708), Point(x=-8.592087420587644, y=-1.9504765782379823), Point(x=-8.581472904853793, y=-1.2774425984488178), Point(x=-8.57522649710059, y=-0.6376558084081717), Point(x=-7.76270298284617, y=-0.7051738267657615), Point(x=-6.989133521275462, y=-0.7705829742609126), Point(x=-6.2361409939771, y=-0.8312967271022609), Point(x=-5.461397449754575, y=-0.8861911213540661), Point(x=-5.516668972001176, y=-0.14714653857259985), Point(x=-5.546556180567529, y=0.5635272047843729), Point(x=-5.53015125668966, y=1.2758376914718037)], id='8dc65409-bfa3-4497-bc39-f6bad3a27643', type='sand', color='SandyBrown', closed=True), Area(outline=[Point(x=-9.385116608117745, y=-12.85754030448829), Point(x=-9.474295969841748, y=-12.178735517627585), Point(x=-9.542503011080386, y=-11.452596676698464), Point(x=-9.56540229955414, y=-10.619257929678904), Point(x=-9.550382372781206, y=-9.640404344363859), Point(x=-10.317089391513765, y=-9.161595132511462), Point(x=-10.766800435647152, y=-8.876664819094996), Point(x=-11.270040883546937, y=-8.537235244630839), Point(x=-11.843026722756182, y=-8.126882370097595), Point(x=-12.509097535035254, y=-7.6252718687168874), Point(x=-12.886929572985853, y=-7.331583289820173), Point(x=-13.301628343516132, y=-7.003007627338227), Point(x=-13.760012655989424, y=-6.633274915629085), Point(x=-14.270672795449656, y=-6.214388575376331), Point(x=-14.235161352770746, y=-7.031114269383393), Point(x=-14.22379239268684, y=-7.720348855408831), Point(x=-14.229729241945268, y=-8.330189246960039), Point(x=-14.248135545922452, y=-8.892784823890945), Point(x=-14.27604571826221, y=-9.42834812819746), Point(x=-14.31153948416658, y=-9.950398046992635), Point(x=-14.353224096285718, y=-10.468657665259485), Point(x=-14.399928611890266, y=-10.990519262243524), Point(x=-14.450518786379618, y=-11.521760215780832), Point(x=-14.503771206275667, y=-12.066821930518278), Point(x=-14.558311351968413, y=-12.628816570188105), Point(x=-14.612619288813953, y=-13.209388928441408), Point(x=-14.66511627122652, y=-13.808547547998073), Point(x=-14.714346214978725, y=-14.424639458779682), Point(x=-14.75920159460002, y=-15.05459850284791), Point(x=-14.799133250652215, y=-15.694528161121164), Point(x=-14.226572929850935, y=-15.691751021326839), Point(x=-13.709506579071038, y=-15.688788758039152), Point(x=-12.814760140874803, y=-15.682171660536538), Point(x=-12.071717397075343, y=-15.674650098535816), Point(x=-11.449388421306455, y=-15.666509738257606), Point(x=-11.157219170072091, y=-15.303208996362518), Point(x=-10.616556226733373, y=-15.267810839810172), Point(x=-10.50292085076253, y=-15.830517144968882), Point(x=-10.505098568418568, y=-16.34225659935484), Point(x=-10.508104281020152, y=-16.8539035410208), Point(x=-10.510020645990204, y=-17.367432757796074), Point(x=-10.508731494672013, y=-17.887369778268656), Point(x=-10.49021398892295, y=-18.425585301585045), Point(x=-10.461924250115965, y=-18.990068623213705), Point(x=-10.417812828181157, y=-19.60406053719565), Point(x=-10.346575753410042, y=-20.31188662208379), Point(x=-9.655371749876533, y=-19.73380737190798), Point(x=-9.057025660636988, y=-19.295534590160873), Point(x=-8.539132164026658, y=-18.930693309948474), Point(x=-8.151737767793914, y=-18.559732610836672), Point(x=-8.26565101672921, y=-17.807937066422333), Point(x=-8.43812142964734, y=-17.055227370583452), Point(x=-8.536610914725802, y=-16.38588498181653), Point(x=-8.593879929146205, y=-15.755447046255233), Point(x=-9.2020380509706, y=-15.725119632811754), Point(x=-9.258489622024557, y=-15.068011380623519), Point(x=-8.986004058221688, y=-14.81449110420975), Point(x=-8.993263433816816, y=-13.910951014265754), Point(x=-8.98774676075423, y=-13.024336104023307)], id='8965b798-ce2c-49a4-a2cd-65cda10092b9', type='path', color='DodgerBlue', closed=True), Area(outline=[Point(x=-10.49562910973494, y=-15.133207134826673), Point(x=-11.174851756911842, y=-15.139970884494446), Point(x=-12.038870262733399, y=-15.14754071965303), Point(x=-12.560681351724678, y=-15.151550426267946), Point(x=-13.156503098847576, y=-15.155712157131031), Point(x=-13.840223639902248, y=-15.160077023790233), Point(x=-14.629939414460338, y=-15.164753805496956), Point(x=-14.578464465336165, y=-14.584898768080588), Point(x=-14.52360905170072, y=-14.017239374018407), Point(x=-14.466158688331317, y=-13.464395140475498), Point(x=-14.407170759367055, y=-12.92805094447778), Point(x=-14.347822433322044, y=-12.4087145883721), Point(x=-14.289267430571105, y=-11.90567658401561), Point(x=-14.17859731265178, y=-10.940185517095937), Point(x=-14.08201568657459, y=-10.005449237630437), Point(x=-14.005450750435237, y=-9.058963107293643), Point(x=-13.977013282780824, y=-8.560816309530695), Point(x=-13.95719772963217, y=-8.02843568403826), Point(x=-13.948666628166839, y=-7.440052442517758), Point(x=-13.955387780988033, y=-6.762669014251621), Point(x=-13.516228194225414, y=-7.1046245584265355), Point(x=-13.116749606147916, y=-7.411118011967003), Point(x=-12.414725877253057, y=-7.937172868051072), Point(x=-11.814314463541525, y=-8.370956455489981), Point(x=-11.291016127508005, y=-8.732553666228224), Point(x=-10.826863001375804, y=-9.03598506789896), Point(x=-10.408395105588923, y=-9.29202372307519), Point(x=-9.676608761311316, y=-9.706451936220057), Point(x=-9.698942792972366, y=-10.70382172622675), Point(x=-9.663308300032734, y=-11.525742705218343), Point(x=-9.589237013509416, y=-12.256057475966777), Point(x=-9.497702904394849, y=-12.947430227252227), Point(x=-9.084419150215085, y=-13.053303993545008), Point(x=-9.057576001756889, y=-13.826925901795768), Point(x=-9.023172862728451, y=-14.605382266805055), Point(x=-9.190403610531652, y=-14.924086643022259), Point(x=-9.760555155298071, y=-15.014493018117129)], id='df86f37a-3b6e-4013-a88a-5bb408e0a12c', type='sand', color='SandyBrown', closed=True), Area(outline=[Point(x=-1.287260901129565, y=1.0593027762754978), Point(x=-2.1638730679131726, y=0.8450675564364918), Point(x=-2.966804057011228, y=0.6504248660702681), Point(x=-3.7048328995122644, y=0.46844853265186215), Point(x=-4.38548490006665, y=0.30170733703316444), Point(x=-5.02401526657599, y=0.15818352382036682), Point(x=-5.650131042575827, y=0.050112871509620405), Point(x=-6.323248817453685, y=-0.00031569021000077413), Point(x=-7.140418687513852, y=0.03376074867227574), Point(x=-6.838947763371432, y=-0.6531325613768453), Point(x=-6.609551559488159, y=-1.2383842865005166), Point(x=-6.046879905232382, y=-1.668943559872823), Point(x=-5.5966351814882245, y=-2.0512469924454875), Point(x=-5.410969912968727, y=-2.6285880683101133), Point(x=-5.242945411994709, y=-3.1634243935035076), Point(x=-5.148648099549396, y=-4.0395649667855125), Point(x=-5.141207972102087, y=-4.544246966494101), Point(x=-5.19273523631192, y=-5.095657236337112), Point(x=-4.546799661762395, y=-5.020098246314062), Point(x=-3.8092306448091517, y=-5.026390130458623), Point(x=-3.047909553562837, y=-5.078042478638135), Point(x=-2.224750145749275, y=-5.153287857284802), Point(x=-1.3071467180388647, y=-5.26535948987315), Point(x=-1.2582996613709794, y=-4.407554998904593), Point(x=-1.1561694662206214, y=-3.4811100106975186), Point(x=-1.18382687853265, y=-2.8189489203666582), Point(x=-1.223174359717008, y=-2.0817150136117695), Point(x=-1.2349818079616117, y=-1.3907233593640458), Point(x=-1.2539757927723274, y=-0.6459655589484865), Point(x=-1.2743328634591542, y=0.16116977759049167)], id='45498502-d9a5-498e-9949-62cd81435a02', type='path', color='DodgerBlue', closed=True), Area(outline=[Point(x=-6.306992626688572, y=-0.21429028747270085), Point(x=-5.911719064727061, y=-1.0968411836294192), Point(x=-5.624181007786829, y=-1.8416990183611863), Point(x=-5.384362753845382, y=-2.510750877026059), Point(x=-5.169773966088477, y=-3.1246485805148163), Point(x=-5.057635839473144, y=-4.071511698900308), Point(x=-5.053845443867929, y=-5.009044299657501), Point(x=-4.699780054496152, y=-4.9400767075831045), Point(x=-3.951871806849954, y=-4.936676280169919), Point(x=-3.186360172800673, y=-4.982363346461711), Point(x=-2.359819819828501, y=-5.049004078252265), Point(x=-1.4451096297735349, y=-5.142277710350761), Point(x=-1.363172275674151, y=-4.347326641000169), Point(x=-1.2372609016349347, y=-3.4735188097265337), Point(x=-1.2858986603879279, y=-2.6170461496432207), Point(x=-1.3581848312510079, y=-1.6393895845179178), Point(x=-1.4047609957098126, y=-1.1080882220249388), Point(x=-1.456545822544811, y=-0.5490081905993365), Point(x=-1.5111435030069291, y=0.04192678520651594), Point(x=-1.5657809381302485, y=0.6750242070938608), Point(x=-2.2816936438144553, y=0.5245272369063549), Point(x=-2.9499602860451044, y=0.3817256849291022), Point(x=-3.573195366588032, y=0.24417813834775082), Point(x=-4.154094920794899, y=0.11387289074417017), Point(x=-4.699345749114114, y=-0.004598467164043996), Point(x=-5.221921499031326, y=-0.10519551096669844), Point(x=-5.744146792935371, y=-0.17979507757803725)], id='56f28f1b-fa89-4732-9379-fea8475ce9a3', type='sand', color='SandyBrown', closed=True)], obstacles=[Obstacle(id='f6d7c650-5125-44f8-82d6-b9b40eccb997', outline=[Point(x=-4.427200041919283, y=-1.3406832522841916), Point(x=-4.39998826476951, y=-0.7479072377406849), Point(x=-3.9173841537122205, y=-0.7783088319732084), Point(x=-3.825471481181808, y=-1.220967309485487)]), Obstacle(id='7bbf7eab-7858-4c2b-b373-242aa51b8e1a', outline=[Point(x=-4.346493054948253, y=-0.6200730106078942), Point(x=-3.8862309804369954, y=-0.5786103328219836), Point(x=-3.901740716883011, y=-0.9142647476858192), Point(x=-4.398744603137223, y=-0.9828700195947908)])], start=Pose(x=-5.253939381970001, y=0.233038071710709, yaw=115.97625986139263, time=1732181946.1662147), goal=Pose(x=-7.054102842279564, y=-7.103207432590725, yaw=0.5557241155229528, time=0))
