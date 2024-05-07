// #include "function.h"

// int main(int argc, char** argv) {
// // Initiate ROS

// std::vector<std::vector<Point>> lines = {
// {{ 95.30850472, 305.96098195},
//  { 96.27119108, 310.10513402},
//  { 97.22997094, 312.87895558},
//  { 98.12537067, 314.9789705 },
//  { 99.14544801, 317.03633076},
//  {100.92641466, 320.09614793},
//  {102.95176609, 323.05511056},
//  {106.05455645, 326.92367816},
//  {114.04905678, 334.85183509},
//  {119.04340138, 338.94277358},
//  {123.72122501, 342.39823062},
//  {127.17309421, 344.77185124}}, 
// {{127.24766252, 344.67458613},
//  {124.7661964 , 342.64735726},
//  {123.78254565, 341.85808371},
//  {120.08237106, 338.90240968},
//  {115.18937502, 334.88341224},
//  {104.9566933 , 324.95858799},
//  { 98.93337585, 316.17863128},
//  { 97.93716926, 314.04496704},
//  { 97.10461095, 311.85196469}}, 
// {{101.60532648, 303.65527343},
//  {100.25664328, 305.39070157},
//  { 99.60810805, 307.26999005},
//  { 99.85852923, 310.76462012},
//  {100.63292203, 312.86240313},
//  {101.74882249, 314.97627491},
//  {105.26915205, 319.9403046 },
//  {108.77425678, 324.20200177},
//  {109.50602533, 325.1153685 },
//  {110.74021427, 326.82306194}}, 
// {{110.91352978, 327.02745328},
//  {109.41679975, 324.97524463},
//  {108.64040376, 323.97376692},
//  {102.08980432, 315.0898414 },
//  {100.93929481, 313.01389307},
//  { 99.99530055, 310.87839567},
//  { 99.00486701, 307.04140504}}, 
// {{271.9874943 , 298.87826208},
//  {273.99804581, 301.32806535},
//  {279.11186742, 303.46954723},
//  {284.75948603, 304.54605822},
//  {286.84832255, 305.31248071},
//  {289.32709706, 306.47052557},
//  {292.14265843, 307.76715274},
//  {297.86872198, 310.10217121},
//  {304.81093498, 312.36347576},
//  {312.01649781, 313.98567744},
//  {316.23396711, 314.59293668},
//  {325.9923955 , 316.09995114},
//  {329.78539565, 317.25768361},
//  {331.84693291, 318.18886195},
//  {334.24462673, 319.60794628},
//  {336.17244575, 320.86141312},
//  {340.78530256, 323.24485886},
//  {345.06780742, 323.92293204}},
// {{333.08749043, 317.74802916},
//  {329.94202558, 317.14156001},
//  {325.87992649, 316.40408804},
//  {316.11819414, 314.6484166 },
//  {312.01481254, 313.83424019},
//  {304.96269978, 312.19162569},
//  {298.04566097, 310.11578159},
//  {294.9882879 , 308.99383435},
//  {291.97888625, 307.75243508},
//  {284.78829787, 304.66375053},
//  {280.38196685, 303.11835777},
//  {277.82888636, 302.43745393},
//  {275.97166972, 301.88670944},
//  {274.01119511, 301.0637176 }},
// {{253.71065089, 297.91441862},
//  {260.51307757, 299.84603029},
//  {262.39626226, 301.22334133},
//  {265.27086248, 305.42740786},
//  {265.86362437, 312.63431845},
//  {264.82851256, 316.61199489},
//  {263.29935116, 320.01899132},
//  {261.29869055, 323.21195428},
//  {257.99831812, 327.17976686},
//  {251.64703155, 332.82635419},
//  {248.17544842, 335.33223449},
//  {245.99817006, 336.77318742}},
// {{245.94927358, 336.9204199 },
//  {249.15221996, 335.2501974 },
//  {252.85658496, 332.75163715},
//  {258.00172139, 328.01304063},
//  {261.08721451, 324.12092273},
//  {262.97908854, 321.05623089},
//  {264.02733988, 318.97400286},
//  {264.91315843, 316.84366012},
//  {266.03339875, 313.06988833}},
// {{199.04393512, 285.82749398},
//  {200.94826396, 287.47254093},
//  {200.85578742, 289.63541987},
//  {199.15910758, 291.62180396},
//  {198.12747004, 292.50450425},
//  {194.87764498, 294.90741128},
//  {192.97292302, 296.02861954},
//  {190.9524701 , 296.97256779},
//  {187.04066575, 298.15915153},
//  {180.08651321, 297.61927933},
//  {175.83950191, 293.64913695},
//  {173.09184845, 294.30436018},
//  {171.20213482, 295.44524303},
//  {168.57168204, 296.70170775},
//  {166.3716282 , 296.23008524},
//  {165.85842339, 293.92067439}},
// {{150.9827549 , 293.09771674},
//  {153.03680877, 294.81050988},
//  {159.90572347, 298.00998352},
//  {162.07441016, 298.31020597},
//  {165.06417901, 298.00675792},
//  {168.76964641, 296.34529714},
//  {171.27155003, 295.12412732},
//  {174.00562807, 294.57327145},
//  {176.57222414, 295.2100332 },
//  {178.30929498, 296.45540919},
//  {180.1261001 , 297.76311942},
//  {186.79015966, 298.57303228},
//  {193.002607  , 296.68103329},
//  {196.24846937, 294.98799179},
//  {199.68465167, 291.9229652 },
//  {201.21348976, 289.2612139 },
//  {201.94230252, 286.86733178}},
// {{215.98357801, 281.91180147},
//  {219.04467444, 279.48938912},
//  {220.01800533, 278.64394826},
//  {222.88257338, 275.82433778},
//  {225.0942773 , 273.11214364},
//  {226.98090023, 270.09083074},
//  {227.99599131, 267.92754899}},
// {{114.57506705, 268.19088319},
//  {115.97657844, 269.84904785},
//  {116.78272466, 270.83763355},
//  {120.84187686, 276.10016401},
//  {124.55213481, 281.11069263},
//  {127.52540925, 285.08024549},
//  {128.85172275, 286.80198342},
//  {135.89448618, 295.02934986}},
// {{136.08341951, 294.90988182},
//  {132.71539901, 291.23349886},
//  {128.71478303, 285.77037477},
//  {127.543657  , 284.02091619},
//  {124.85732437, 280.02723062},
//  {121.10285349, 275.06728409},
//  {115.98256359, 269.97081364}},
// {{239.00251824, 261.99775258},
//  {238.94880163, 281.83628468},
//  {240.00527589, 285.16479319},
//  {241.09321945, 287.32146676},
//  {246.95378066, 292.50099105},
//  {249.92297545, 293.75029626},
//  {259.12113216, 296.30574579},
//  {263.04502455, 297.70995673},
//  {267.94798506, 301.64307306},
//  {268.90430679, 303.65677252},
//  {268.99999222, 314.8536935 },
//  {268.02373994, 318.0033964 },
//  {267.08358138, 320.15618412},
//  {265.97935804, 322.18730086},
//  {264.02083833, 325.10712564},
//  {260.93715735, 328.86305942},
//  {253.98961941, 335.80502732},
//  {250.03491209, 339.02347707},
//  {246.98614065, 341.16192673},
//  {239.99964071, 344.95167633}},
// {{242.01656219, 344.00734013},
//  {247.93158433, 341.0111696 },
//  {251.03888903, 339.0000215 },
//  {255.04124421, 335.90875672},
//  {260.99263482, 329.99396349},
//  {263.9943154 , 326.13566536},
//  {265.89255867, 323.16586692},
//  {266.98518927, 321.09751081},
//  {267.95325591, 318.8781611 },
//  {269.28694792, 314.64852035},
//  {268.95088837, 302.85993863},
//  {267.82264705, 301.04982793},
//  {263.86713478, 297.88102994},
//  {260.24026007, 296.42960582},
//  {254.05388884, 295.02084271},
//  {250.03795792, 293.87014024},
//  {247.88879031, 292.87174838},
//  {241.03883452, 286.26457797},
//  {239.91463584, 284.11235484},
//  {239.05178057, 281.79295756}},
// {{237.189217  , 252.0944368 },
//  {233.29997499, 255.54308706},
//  {231.17108129, 257.44573419},
//  {229.83186191, 259.21998307},
//  {229.34148576, 260.70698118},
//  {229.29310166, 262.00859284},
//  {229.43271035, 263.87220007},
//  {229.44056704, 266.10898479}},
// {{181.00831454, 236.22324413},
//  {180.06592296, 236.39577024},
//  {177.79797902, 236.7509101 },
//  {173.19312099, 237.31053701},
//  {171.02704716, 237.52097237},
//  {162.82007697, 237.90320569},
//  {156.0305844 , 237.08578115},
//  {154.15207979, 236.47831093},
//  {151.90487417, 235.33126839}},
// {{115.23800523, 225.05253392},
//  {115.96904378, 228.62492275},
//  {115.67028944, 230.50419898},
//  {115.47769805, 232.92440411},
//  {115.95166867, 234.80529396},
//  {118.29851185, 238.13007871},
//  {120.39478299, 239.95856758}},
// {{126.62708631, 223.38893893},
//  {129.32810438, 225.2934405 },
//  {130.337302  , 226.0869634 },
//  {136.74939991, 232.0656956 },
//  {137.8997107 , 233.29397528},
//  {144.56815202, 241.18661938},
//  {146.49138242, 243.62204946},
//  {149.11371432, 246.59062764},
//  {150.03336054, 247.45855692},
//  {153.85178741, 250.01313289}},
// {{154.11786159, 249.88566922},
//  {150.45273601, 247.45228924},
//  {148.32608251, 245.73676378},
//  {147.3929115 , 244.70995365},
//  {146.19598115, 242.98875126},
//  {144.3786947 , 240.28813426},
//  {132.59842665, 226.47875912},
//  {133.66033256, 225.61613256},
//  {138.64583136, 225.88163059},
//  {141.16697323, 226.91734169},
//  {143.06416874, 229.04457462}},
// {{144.00852869, 232.00198461},
//  {145.00708621, 232.79580967},
//  {145.92903245, 233.47837533},
//  {148.15727275, 234.39288206},
//  {150.76401321, 234.5284027 },
//  {153.19548537, 236.77750419},
//  {162.8695926 , 238.0403641 },
//  {171.11990208, 236.8946718 },
//  {172.98855466, 237.14333579},
//  {177.92993572, 237.91192538},
//  {182.0457863 , 236.00715853},
//  {187.99074908, 235.2774522 },
//  {190.87439466, 235.31614035},
//  {193.32248852, 234.69490413},
//  {194.72238955, 235.85528904},
//  {192.04642349, 238.59355868},
//  {187.06955266, 240.49870968},
//  {180.94751257, 240.74099913},
//  {171.01676595, 240.07499929},
//  {166.99453347, 241.97553335}},
// {{157.57165996, 244.67938283},
//  {157.66620145, 242.98268474},
//  {158.97045439, 241.84854263},
//  {166.45119626, 241.12019772},
//  {172.38293964, 240.26284529},
//  {182.96656428, 240.30082613},
//  {185.99098402, 242.80552066}},
// {{183.10621897, 244.10831958},
//  {185.00388647, 243.24166152},
//  {185.61533549, 242.19772665},
//  {188.55324992, 239.9104669 },
//  {191.84365536, 238.60157239},
//  {193.62072449, 237.62280978},
//  {195.35858299, 235.4663306 },
//  {195.91353514, 233.86680131},
//  {199.98481117, 231.98431127}},
// {{182.07257302, 233.65015432},
//  {183.97978565, 232.61047783},
//  {186.77751445, 231.03724592},
//  {192.20544314, 228.41107147},
//  {198.2452792 , 227.22253592},
//  {200.57413792, 228.22041092},
//  {202.14526662, 231.84810361}},
// {{201.99843317, 226.94777718},
//  {194.00782945, 228.23418536},
//  {185.96177249, 231.54880492},
//  {182.05917797, 233.05127277},
//  {176.96754297, 234.34207883},
//  {163.00849987, 233.64308295},
//  {158.99691423, 232.53157364},
//  {157.02414343, 232.54427905},
//  {153.84404411, 231.18221952},
//  {152.26781643, 231.13698269},
//  {150.70579521, 233.70399374},
//  {149.29323966, 234.977374  },
//  {146.65327399, 233.45692376},
//  {144.33896749, 231.50780765},
//  {143.09276367, 230.0065585 },
//  {141.51665192, 227.41561595},
//  {139.31337102, 225.72681644},
//  {131.91505267, 227.07127167},
//  {130.03471024, 225.97138138}},
// {{123.1129173 , 208.82421683},
//  {121.84451699, 209.26501272},
//  {118.99651606, 211.89955663},
//  {118.06303954, 213.99776601},
//  {118.04445157, 221.94880915},
//  {118.87779956, 224.12745663},
//  {120.06075898, 225.93718203}},
// {{120.57179977, 226.0582582 },
//  {119.17816864, 224.06123011},
//  {117.93526864, 220.6569658 },
//  {118.40768272, 215.24107904},
//  {119.3214135 , 213.20319732},
//  {123.23982008, 208.55196626},
//  {125.14414462, 207.17249829},
//  {127.20170202, 206.05480498}},
// {{175.24032252, 207.38117512},
//  {177.53525831, 206.26386768},
//  {183.21482699, 202.40065617},
//  {185.27696286, 201.29152182},
//  {190.52876683, 200.76209688},
//  {192.03768524, 201.49011655},
//  {194.16617724, 203.41056578}},
// {{194.00202303, 203.20288485},
//  {192.1009572 , 203.91071194},
//  {190.74806985, 204.27780074},
//  {188.17178049, 204.87236323},
//  {186.3100076 , 205.38886143},
//  {184.51043126, 206.11273205},
//  {182.94453464, 207.00843013},
//  {181.33863599, 208.29109134},
//  {179.87355994, 209.9351243 }},
// {{145.91314684, 200.18993573},
//  {148.43264412, 203.91029101},
//  {150.72157891, 204.3203358 },
//  {152.62851233, 203.89269696},
//  {155.18073321, 203.01374136},
//  {159.96331176, 202.11292586},
//  {162.38125155, 202.54257279},
//  {168.46544293, 207.99590813},
//  {168.37200922, 210.92154334},
//  {166.94136913, 213.10004902}},
// {{169.95443707, 207.32290055},
//  {166.16268779, 204.24144965},
//  {160.73959784, 202.36114501},
//  {154.31168364, 202.84002427},
//  {150.78944879, 203.95709647},
//  {149.0082003 , 203.93307689},
//  {147.03394457, 202.34430716}},
// {{202.91693545, 196.05049453},
//  {207.06415916, 199.94345688},
//  {211.40040503, 202.49535219},
//  {213.05681694, 203.60260762},
//  {215.16742411, 206.14181307},
//  {215.48904869, 207.76526582},
//  {215.53197898, 209.14025347},
//  {215.70351116, 211.75084157},
//  {217.66972047, 215.10991487}},
// {{217.81140765, 215.01331506},
//  {215.92273754, 210.01639864},
//  {216.48931345, 207.69871188},
//  {216.18514627, 205.51133365},
//  {213.72085842, 202.70478429},
//  {208.96538745, 199.94736528},
//  {205.83566281, 198.30564179},
//  {205.06948642, 197.80244941}},
// {{210.15128698, 192.86574028},
//  {211.50069846, 195.28617127},
//  {212.33451795, 197.11177875},
//  {214.10524767, 199.78011898},
//  {215.94291554, 201.73948689},
//  {225.84268394, 211.37183755},
//  {227.06781062, 213.29110103},
//  {228.0057349 , 215.97163709},
//  {228.05117555, 217.8315609 },
//  {227.18918673, 220.66322822},
//  {225.6396653 , 223.55949347},
//  {224.11069388, 226.21151057},
//  {223.070529  , 228.38459185},
//  {222.09578452, 232.13770901},
//  {221.58921752, 236.6132516 },
//  {221.37248811, 238.12185963},
//  {220.93036334, 240.05892291}},
// {{223.71387476, 227.42356087},
//  {226.02793061, 224.68167832},
//  {227.55619901, 221.69004145},
//  {228.12896434, 218.75057402},
//  {227.90469052, 216.00648094},
//  {226.62297562, 212.43149403},
//  {224.75813146, 209.39660449},
//  {217.361732  , 201.48725153},
//  {213.24296957, 197.78666518},
//  {211.68253212, 196.34564918}},
// {{253.24600827, 181.13365944},
//  {251.49834704, 184.608907  },
//  {250.91183524, 186.13101618},
//  {249.47805128, 191.20652011},
//  {248.29121875, 198.97316356},
//  {247.98610626, 202.86875644},
//  {247.82858107, 206.07236481},
//  {247.75985209, 208.00561246}},
// {{248.00169608, 207.98809818},
//  {247.95731047, 201.09731963},
//  {248.05845708, 198.90122242},
//  {249.01365083, 192.01841614},
//  {249.88920909, 188.96400105},
//  {251.11068352, 186.05083248},
//  {252.96899294, 182.9801101 }},
// {{263.87656959, 171.0510492 },
//  {265.57405633, 176.47458957},
//  {266.59990825, 179.74798957},
//  {266.53397338, 184.80306171},
//  {263.70143806, 185.4053584 },
//  {261.73216247, 184.56072474},
//  {261.98189193, 174.95722681}},
// {{256.98260241, 167.99083077},
//  {258.06205306, 175.02734873},
//  {256.71988407, 186.91449312},
//  {255.51444778, 193.13924001},
//  {254.26945935, 201.830653  },
//  {253.6489734 , 207.10465919},
//  {252.75534418, 214.00538909},
//  {250.12932556, 227.98715725},
//  {249.04077476, 232.00971304},
//  {247.78327444, 235.95363187},
//  {246.00190749, 241.00610532},
//  {244.13228045, 246.08615405},
//  {240.15562922, 254.94361241},
//  {238.96724505, 256.98304251},
//  {237.67770168, 258.94232447},
//  {233.20302351, 265.11338333},
//  {223.79499194, 279.85171634},
//  {221.45913052, 283.10313725},
//  {220.68668648, 284.06862309},
//  {219.01526462, 285.93878514}},
// {{217.10429539, 287.11004466},
//  {220.731023  , 284.70631828},
//  {224.18102145, 281.21586827},
//  {232.08579371, 269.01079347},
//  {236.92514386, 260.99605491},
//  {238.76103925, 257.8302915 },
//  {243.16864621, 249.1193531 },
//  {245.249838  , 244.1223046 },
//  {247.01999253, 239.03636215},
//  {247.92192515, 235.9259374 },
//  {248.88790969, 231.92311139},
//  {249.71150684, 227.89298167},
//  {251.40546086, 219.13522774},
//  {253.37805643, 210.02429689},
//  {253.88861921, 207.01222253},
//  {254.37205187, 201.87490326},
//  {255.3260952 , 193.10335486},
//  {256.8747737 , 186.95850688},
//  {258.00680765, 175.00206644}},
// {{169.04521031, 166.00525095},
//  {169.75040775, 170.94468734},
//  {168.67932294, 176.2303857 },
//  {166.99263863, 180.56258317},
//  {164.77568707, 184.39684867},
//  {163.5438343 , 190.67902024},
//  {163.14195001, 192.1665378 },
//  {162.07094899, 194.01468613}},
// {{162.12886598, 194.07481007},
//  {164.58615761, 190.75804814},
//  {166.03174619, 184.64911559},
//  {166.63138734, 182.6607714 },
//  {167.28759844, 180.68303599},
//  {168.40767499, 176.21326844},
//  {169.92656945, 170.96095037}},
// {{ 85.45631071, 165.13414086},
//  { 84.41454982, 168.76948381},
//  { 83.4496046 , 170.8008016 },
//  { 82.21091329, 173.22219343},
//  { 81.35151464, 175.18806387},
//  { 80.83783073, 177.00676237},
//  { 80.68347506, 179.05167605},
//  { 81.25882031, 182.71741311},
//  { 82.33937171, 186.05444227},
//  { 83.48018122, 189.07181852},
//  { 84.5174279 , 191.98320412}},
// {{ 83.98971818, 191.99828325},
//  { 84.04269912, 187.99071396},
//  { 82.90829378, 185.05084408},
//  { 81.29630804, 181.90422905},
//  { 80.55506441, 180.05353397},
//  { 80.2375972 , 178.00794469},
//  { 81.97031927, 173.994451  }},
// {{187.76731078, 160.01425306},
//  {188.68184174, 161.8573414 },
//  {189.34598138, 162.80300334},
//  {190.28448168, 163.8483604 },
//  {191.14580492, 164.61328041},
//  {193.22732415, 165.98632021},
//  {195.87427206, 167.13750763},
//  {199.67147474, 168.10762443},
//  {203.78378746, 168.58847321},
//  {207.93253591, 168.66368029},
//  {215.98197036, 167.97017107},
//  {217.59454571, 167.71998167},
//  {222.7086691 , 166.69000288}},
// {{222.94130587, 166.87476025},
//  {217.35031793, 167.5003939 },
//  {215.71810755, 167.75125264},
//  {207.95027987, 168.76289249},
//  {204.00678366, 168.84310394},
//  {200.00576539, 168.34822721},
//  {196.10706023, 167.05602899},
//  {193.9203795 , 165.86334058}},
// {{140.2081287 , 155.74221961},
//  {142.40076518, 155.85610067},
//  {145.50176197, 156.91819847},
//  {148.07335742, 158.40407937},
//  {149.96093596, 160.19130999},
//  {150.65749304, 161.93268942},
//  {150.19755774, 163.95540247}},
// {{125.77126302, 157.42042066},
//  {124.66951417, 153.98641462},
//  {123.3546047 , 152.34048688},
//  {122.16281369, 152.46212571},
//  {116.93887955, 156.95914155},
//  {115.22562296, 157.5337075 },
//  {113.8773019 , 156.29770308}},
// {{122.81842154, 151.8544794 },
//  {124.50397512, 156.41106785},
//  {127.81740307, 157.89103682},
//  {129.69860634, 157.63511094},
//  {131.77020174, 156.82000186},
//  {134.61110588, 154.8653641 },
//  {135.92290653, 153.50120643},
//  {136.85737978, 152.02173261}},
// {{210.81685559, 153.87300285},
//  {210.52142178, 156.43412351},
//  {211.7679957 , 156.7357191 },
//  {213.42707287, 155.60272141},
//  {214.56141744, 154.06279261},
//  {215.92147625, 152.79609418},
//  {219.01467625, 154.21468457},
//  {224.91667321, 155.48456826},
//  {228.06163887, 156.77469755},
//  {233.99077203, 157.02159596}},
// {{233.99671699, 157.01609669},
//  {228.05854048, 156.70817363},
//  {224.8424826 , 155.75937188},
//  {220.26712715, 153.95097186},
//  {216.60960968, 152.77734218},
//  {214.4552218 , 152.99537844},
//  {213.72041958, 154.71451158},
//  {212.04988172, 157.07815372}},
// {{140.97241622, 146.96818053},
//  {143.04602572, 147.08761541},
//  {145.04831555, 147.90131678},
//  {146.87392827, 149.11771777},
//  {149.02723435, 150.68665432},
//  {150.06322708, 151.3007122 },
//  {151.96885282, 151.937803  }},
// {{108.24994853, 156.18885548},
//  {111.47893969, 154.54482646},
//  {113.55892131, 152.96118569},
//  {114.50522969, 152.1394998 },
//  {116.48405089, 150.32031658},
//  {118.9896976 , 148.08092496},
//  {120.71504929, 146.75089859},
//  {138.018163  , 147.01349244}},
// {{216.07836922, 145.90090779},
//  {218.89691316, 147.06501721},
//  {220.8669498 , 148.27105034},
//  {224.23081432, 150.74926048},
//  {227.07206248, 152.79475224},
//  {228.9582694 , 153.88230949},
//  {230.72543735, 154.58510608},
//  {232.13759815, 154.88118834},
//  {234.03358613, 154.87040801}},
// {{178.4332917 , 144.91528164},
//  {179.46817594, 148.14629977},
//  {180.41688656, 150.19791427},
//  {182.60580472, 153.43934504},
//  {184.20382261, 155.03296918},
//  {186.06056841, 156.35173752},
//  {190.09684632, 157.87785038},
//  {193.74822391, 157.92687968},
//  {195.11285058, 157.50013425},
//  {196.60789737, 156.50813979},
//  {198.2456319 , 153.10344848}},
// {{222.0568941 , 152.92601474},
//  {218.73826765, 150.27553116},
//  {216.99392766, 149.13302686},
//  {215.36627743, 148.25951953},
//  {212.98817791, 147.27983864},
//  {209.96425422, 146.39784162},
//  {205.81528624, 145.70776829},
//  {196.84471458, 146.74962844},
//  {194.22037174, 148.30476956},
//  {191.62095998, 151.27033495},
//  {190.28500724, 153.80047827},
//  {188.78735093, 155.62492886},
//  {187.16841445, 156.11662857},
//  {184.30425714, 155.27716156},
//  {181.84583874, 152.87652896}},
// {{ 77.07299652, 145.07842564},
//  { 77.80761476, 147.04449375},
//  { 79.08525834, 148.74775521},
//  { 81.0728009 , 150.62780229},
//  { 82.06596896, 151.48584174},
//  { 84.81176872, 154.27663148},
//  { 86.12567755, 158.5329553 },
//  { 84.95791424, 162.20609461}},
// {{ 84.94870266, 162.07990527},
//  { 86.08813393, 158.80963839},
//  { 85.1093253 , 154.11801208},
//  { 83.86721706, 152.15400615},
//  { 82.81586301, 150.88245096},
//  { 79.50184601, 147.80478553},
//  { 78.66891204, 147.15120161}},
// {{270.59764571, 143.77086517},
//  {269.59880411, 145.44330343},
//  {269.24505529, 148.77561211},
//  {270.08900371, 152.97298284},
//  {271.67477078, 158.77031583},
//  {271.95299856, 160.36730004},
//  {271.84172185, 163.89962058}},
// {{154.05707748, 139.90904946},
//  {156.94618786, 141.16144981},
//  {158.75678983, 143.11817951},
//  {160.85319143, 147.39478683},
//  {162.07731438, 150.51655808},
//  {163.33518519, 153.9519788 },
//  {164.71393557, 157.0992471 },
//  {166.41271473, 159.76887116},
//  {171.68757615, 166.15645161},
//  {172.16002738, 168.92342764}},
// {{172.48097702, 168.80718096},
//  {171.15480354, 166.37023132},
//  {166.51664148, 159.6531753 },
//  {163.41208504, 153.03934153},
//  {162.37778799, 150.27133608},
//  {161.58042463, 148.16000724},
//  {160.51529116, 145.61779113},
//  {157.96198915, 141.08093643}},
// {{266.88708164, 139.45558312},
//  {269.23632118, 139.86228817},
//  {272.7647093 , 142.9486457 },
//  {274.08335982, 146.59878757},
//  {275.02578121, 152.89826487},
//  {276.05012071, 162.32208392},
//  {275.9009681 , 180.0024968 },
//  {275.04705806, 183.95695934},
//  {273.7528779 , 187.82646163},
//  {270.4725109 , 195.12717665},
//  {269.18833487, 199.08384123},
//  {268.26045805, 204.89274597},
//  {267.069683  , 208.01308423},
//  {265.19117377, 209.95923097},
//  {263.13129149, 211.0598641 },
//  {255.93827001, 210.99248571}},
// {{268.56807564, 204.91382026},
//  {269.47552158, 199.10761393},
//  {270.4858692 , 196.06166291},
//  {273.67469509, 187.90987019},
//  {274.83084596, 183.99389295},
//  {275.65069979, 179.99759867},
//  {276.31347231, 162.12928253},
//  {275.39120162, 152.76204635},
//  {274.0374992 , 146.83679347},
//  {272.22876595, 142.42637171},
//  {270.84386871, 140.21911229},
//  {269.49948494, 138.64193472}},
// {{216.03614424, 132.03988963},
//  {212.92196108, 133.87772617},
//  {208.04169959, 135.11094351},
//  {193.20803342, 135.42997951},
//  {190.62474247, 135.84462391},
//  {188.09410599, 136.76956097},
//  {186.1700991 , 138.15869083},
//  {184.90321411, 139.76858548}},
// {{184.95274989, 139.85773082},
//  {190.12576152, 136.17502118},
//  {193.96195449, 135.1983365 },
//  {207.7994304 , 134.52190667},
//  {212.88366348, 133.50195812},
//  {216.4485933 , 132.68652887},
//  {222.0547274 , 132.21757005},
//  {225.8010228 , 133.08444611},
//  {227.87271618, 134.02563758},
//  {230.86223127, 135.97124926},
//  {234.52375966, 139.28322376},
//  {236.6414692 , 141.49383435},
//  {238.07192041, 142.98255672}},
// {{235.88949436, 145.20299562},
//  {232.32128075, 144.4248734 },
//  {229.75968168, 143.56232357},
//  {226.17191174, 142.35386112},
//  {222.76442048, 141.55046825},
//  {215.07569331, 141.13808237},
//  {207.04064464, 142.44382516},
//  {200.27872935, 143.61772178},
//  {198.64536123, 143.7458293 },
//  {192.05278246, 142.96001943}},
// {{195.20311598, 138.61354651},
//  {192.6125775 , 140.15222939},
//  {191.90150038, 140.98311123},
//  {192.48233495, 142.92838857},
//  {198.76411496, 144.34033382},
//  {206.99027728, 143.04446417},
//  {215.07043888, 140.91478618},
//  {222.97564007, 141.02314014}},
// {{226.0556485 , 142.94001887},
//  {229.75146567, 143.217098  },
//  {232.1977931 , 144.88038548},
//  {236.29712958, 144.8677455 },
//  {237.80396936, 143.75274413},
//  {237.43998875, 141.73253708},
//  {235.4624102 , 139.54151056},
//  {233.57317236, 137.90066494},
//  {232.73031982, 137.16564578},
//  {231.70899352, 136.2771387 },
//  {230.85887346, 135.57360946},
//  {228.9063557 , 134.19612606},
//  {226.31105935, 132.95386867},
//  {221.90282064, 132.00090675}},
// {{119.99691856, 132.00419449},
//  {111.95374115, 132.99453114},
//  {109.1481797 , 133.98252461},
//  {103.74605264, 138.93528843},
//  {102.7616304 , 141.18126305},
//  {102.77776306, 144.0856585 },
//  {104.84761088, 145.5823835 },
//  {106.60114051, 145.01318243},
//  {111.12589251, 141.48308851},
//  {114.01850366, 140.09035112},
//  {116.06857855, 139.6425709 },
//  {125.9305604 , 140.92875064},
//  {137.09776789, 144.06328939},
//  {141.69972842, 144.44581036},
//  {143.25013327, 144.51253267},
//  {147.97579839, 145.05458026}},
// {{129.11277531, 141.19941539},
//  {126.82664763, 141.70160396},
//  {116.09320668, 140.08106548},
//  {114.03566618, 140.126207  },
//  {111.11769774, 141.15188114},
//  {106.58968622, 144.86393526},
//  {104.9106314 , 145.3754723 },
//  {103.31597426, 144.05198769},
//  {103.08183802, 141.64405001},
//  {109.76611524, 133.81124075},
//  {112.18673752, 132.84618125},
//  {118.81019229, 133.46445338},
//  {121.26283397, 134.48024573},
//  {122.88999755, 135.20226066}},
// {{156.99056961, 145.01232557},
//  {154.05213529, 142.88047179},
//  {149.69069401, 139.80737785},
//  {147.26092554, 138.30646827},
//  {145.31597353, 137.28449454},
//  {142.80068462, 136.30461774},
//  {139.79995327, 135.66014315},
//  {131.05860826, 134.76726868},
//  {127.10627114, 133.8140836 },
//  {123.92418473, 132.16274879}},
// {{ 97.53747956, 118.80187038},
//  { 94.43792986, 124.15041686},
//  { 93.60304508, 126.30696848},
//  { 91.60701437, 135.77945973},
//  { 91.05370213, 143.76027492},
//  { 90.78182378, 151.13216331},
//  { 90.03001712, 162.1063234 },
//  { 89.09618984, 168.05430837},
//  { 87.85279827, 172.90821454}},
// {{ 87.73395669, 172.91778372},
//  { 88.55215634, 169.30651453},
//  { 88.8500283 , 167.74490119},
//  { 89.72121781, 162.00477555},
//  { 90.73564949, 151.20256868},
//  { 91.19825298, 143.82779822},
//  { 91.81429185, 135.78925986},
//  { 93.36278143, 128.15143426},
//  { 94.48415733, 125.27395624},
//  { 96.54750778, 121.78100776}},
// {{230.99556801,  77.97216953},
//  {233.12823359,  80.05692814},
//  {233.80083942,  82.95758645},
//  {232.97293716,  91.1000786 },
//  {234.1316551 ,  95.92728308},
//  {236.49756534, 100.67547889},
//  {238.24518337, 104.44833768},
//  {240.11152269, 111.88081981},
//  {240.32372405, 118.88897433},
//  {240.87961339, 122.07423961},
//  {246.63585239, 133.08495316},
//  {247.65410329, 139.85539752},
//  {248.33133302, 145.12544344},
//  {249.41515154, 149.9311059 },
//  {251.78764791, 157.04060789},
//  {253.07422578, 160.98222257},
//  {254.02318185, 164.99665008},
//  {253.99166211, 173.00172332}},
// {{249.08763309, 149.9795281 },
//  {248.57370806, 145.10649391},
//  {247.69658686, 139.8227732 },
//  {246.47129673, 132.10438054},
//  {244.92002688, 128.07474531},
//  {242.1373279 , 123.98926838},
//  {241.02084889, 121.9515497 },
//  {240.27164731, 118.91972973},
//  {240.01194309, 111.94753684},
//  {238.35530525, 104.3175918 },
//  {236.43322322,  99.78569512},
//  {234.13446739,  94.9648295 },
//  {233.00426369,  90.99947646},
//  {233.43513189,  83.04380179},
//  {233.69190102,  81.06721957},
//  {233.75468875,  79.92538005}},
// {{203.12904505,  67.11703802},
//  {205.80537692,  67.12199754},
//  {208.72277349,  67.17696042},
//  {210.28290719,  67.23277963},
//  {213.1812279 ,  67.39483532},
//  {221.70932246,  68.44592821},
//  {225.19865812,  69.20251178},
//  {228.97068887,  70.30794908}},
// {{228.92379099,  70.3261828 },
//  {227.23936159,  69.51096979},
//  {225.8191758 ,  68.9954172 },
//  {222.00759537,  68.09906047},
//  {214.02184971,  67.42004702},
//  {208.98324483,  67.34413061},
//  {206.00498171,  67.30419211}},
// {{117.09449526,  65.84001225},
//  {115.85345161,  69.48493615},
//  {110.95855975,  72.14726152},
//  {105.2077935 ,  72.68186486},
//  {103.12225786,  73.24370341},
//  {101.82209019,  73.94735991},
//  { 98.77661675,  77.18657588},
//  { 98.10124606,  78.26997674},
//  { 97.06348901,  80.19830928}},
// {{100.37246987,  82.30669092},
//  {100.13602416,  79.83270249},
//  {100.48097168,  77.56765371},
//  {102.01481466,  74.76271606},
//  {104.01656719,  73.31930951},
//  {106.07062068,  72.76293908},
//  {111.86567719,  72.31897522},
//  {113.97892467,  71.80368359},
//  {116.06392991,  70.32532941}},
// {{201.02404478,  65.07793844},
//  {197.93448931,  65.79076406},
//  {192.10680311,  66.3410708 },
//  {186.86288934,  68.45938041},
//  {182.12857239,  69.76310436},
//  {177.87913777,  70.37394575},
//  {173.27997077,  70.66841888},
//  {171.74866721,  70.71900976},
//  {167.06807645,  70.9336159 },
//  {164.95901382,  70.80429009},
//  {155.96574657,  69.07312788},
//  {153.22409475,  69.30202257},
//  {151.77200376,  69.5989678 },
//  {149.04648997,  70.09434331}},
// {{149.08429301,  70.24307914},
//  {151.69554821,  69.34407789},
//  {153.19955822,  69.18320995},
//  {156.05450199,  69.32953411},
//  {164.90275156,  70.87180154},
//  {167.0647273 ,  70.99161155},
//  {171.99861971,  70.03668582}},
// {{173.0144392 ,  71.07212217},
//  {177.92408299,  70.70566289},
//  {182.16686367,  69.56010042},
//  {186.79282176,  68.14614152},
//  {192.16668884,  67.00068722},
//  {197.8671948 ,  65.65533862},
//  {201.08502264,  64.5607079 },
//  {207.04481939,  63.15984958},
//  {208.88985235,  63.40441109},
//  {211.04821437,  64.73497859}},
// {{216.99109186,  51.27969249},
//  {213.87018241,  51.29872944},
//  {212.21711082,  49.30802311},
//  {211.66133422,  47.37394883},
//  {211.30239622,  45.91385213},
//  {210.6261824 ,  43.91435246},
//  {207.69001994,  39.72406804},
//  {205.64168213,  38.18733349}},
// {{211.7858715 ,  28.19788408},
//  {214.50145472,  29.3697558 },
//  {216.09143624,  30.29860107},
//  {217.56307627,  31.36140856},
//  {218.51514284,  32.13678535},
//  {219.28198104,  32.78199971},
//  {220.25512119,  33.58436171},
//  {221.11317052,  34.24733263},
//  {223.89274569,  36.02187108}},
// {{169.87578603,  28.02438516},
//  {172.19945708,  27.00485248},
//  {177.0031733 ,  26.82869565},
//  {180.83551443,  28.18570147},
//  {183.98235196,  30.09729896},
//  {187.25151781,  32.70243825},
//  {188.85219938,  34.15662802}},
// {{156.00497615,   9.99796183},
//  {146.97001695,  10.92725799},
//  {143.03908232,  12.15812441},
//  {138.94937301,  13.91563844},
//  {131.26555006,  18.23539441},
//  {127.65057279,  20.53693916},
//  {122.11096012,  24.18022442},
//  {119.12178368,  26.2539967 },
//  {113.61826041,  30.45607226},
//  {110.26022728,  33.37877532},
//  {102.44992638,  41.25765323},
//  { 99.45467502,  44.60865165},
//  { 93.99580727,  51.00360585},
//  { 89.13141352,  57.06582409},
//  { 87.03393592,  60.09764329},
//  { 82.97080947,  67.91904764},
//  { 81.91038612,  70.99773841},
//  { 79.2954127 ,  87.02085073},
//  { 78.63077308,  93.98213398},
//  { 77.13605774, 100.00646619}},
// {{ 75.97320777, 108.00587367},
//  { 76.24123858, 122.94311333},
//  { 76.92212888, 130.91134533},
//  { 77.27024058, 132.41812858},
//  { 78.89831309, 136.23823337},
//  { 79.95034394, 137.55959552},
//  { 81.74452716, 138.9237102 }},
// {{ 82.00992536, 139.0880741 },
//  { 79.09674537, 140.7731065 },
//  { 76.65568213, 139.86010763},
//  { 74.03055687, 140.32726011},
//  { 72.30815758, 142.21401854},
//  { 71.28966594, 144.88545286},
//  { 71.37030058, 153.66595722},
//  { 71.67268523, 157.9403312 },
//  { 70.88113976, 167.36953304},
//  { 69.72952741, 181.96133435},
//  { 70.61863321, 185.93635157},
//  { 72.31980223, 189.85926959},
//  { 76.03012762, 198.17356754},
//  { 75.98705071, 203.94563575}},
// {{ 77.01428483, 207.01517214},
//  { 79.86450593, 210.12395546},
//  { 83.15045067, 210.66110628},
//  { 89.21275568, 211.28036519},
//  { 92.58584696, 214.98082883},
//  { 93.90076876, 219.01263392},
//  { 95.47857086, 226.86051091},
//  { 96.19666646, 230.89372624},
//  { 96.84913493, 234.03806498},
//  { 97.660676  , 237.18012967},
//  {100.79884196, 245.08457891},
//  {105.63542892, 252.67991872},
//  {108.88196306, 258.07982389},
//  {109.80388577, 260.19059309},
//  {110.86744964, 263.99980251},
//  {111.97170345, 271.82292993},
//  {113.33558691, 282.17822821},
//  {112.55073828, 291.86649428},
//  {110.29486238, 296.04224189},
//  {106.94587855, 299.00889497}},
// {{105.00484223, 299.96699225},
//  {101.91054766, 299.94965981},
//  { 99.25272063, 301.3976379 },
//  { 96.90220622, 302.82874948},
//  { 94.78203096, 303.48508867},
//  { 91.06728676, 304.2764698 },
//  { 88.27843072, 305.51477634},
//  { 84.72915114, 307.47100479},
//  { 81.91619996, 308.62303958},
//  { 79.21552432, 309.679748  },
//  { 75.03936744, 312.007821  },
//  { 72.9197858 , 312.87945447},
//  { 69.9005269 , 313.73286089},
//  { 64.13117903, 315.22901623},
//  { 59.95523217, 316.98829502},
//  { 42.92050341, 325.67381169},
//  { 40.04551742, 326.3378593 },
//  { 37.17413228, 327.23601199},
//  { 34.79703739, 328.59937067},
//  { 32.05777756, 330.12233212}},
// {{ 85.98867284, 307.97698748},
//  { 88.96080674, 304.89987897},
//  { 91.15705087, 304.36911603},
//  { 95.86326907, 303.64188894},
//  { 97.82708453, 302.67158508},
//  {100.14804264, 301.30737744},
//  {102.19695241, 300.52714281},
//  {104.95373654, 299.76484516},
//  {107.71408147, 298.69225458},
//  {109.85623489, 297.07655891},
//  {110.69620354, 296.13722523},
//  {112.50413951, 292.79902165},
//  {113.22043091, 282.30340347},
//  {111.89187792, 271.80692523},
//  {111.01791603, 263.8946272 },
//  {110.07649336, 260.03978109},
//  {108.84649664, 257.15224989},
//  {102.30723433, 246.86407377},
//  { 99.67094777, 242.09560415},
//  { 98.10232801, 236.97945293}},
// {{ 97.11784076, 233.76049112},
//  { 95.80033034, 231.32022221},
//  { 94.99667962, 227.199237  },
//  { 94.20960437, 218.4468927 },
//  { 93.07935652, 214.6815065 },
//  { 91.89224141, 213.10899201},
//  { 89.78206315, 211.91162761},
//  { 83.19241875, 210.55100525},
//  { 80.88826194, 209.86402216},
//  { 76.98497767, 206.18191717},
//  { 76.1985001 , 203.97150606},
//  { 75.71222511, 197.03242805},
//  { 72.30124465, 189.92260203},
//  { 70.8719431 , 186.05877541},
//  { 69.94675675, 181.98983094},
//  { 70.06412669, 166.99756997},
//  { 71.92296766, 158.01029696},
//  { 72.02738682, 153.98553315},
//  { 71.03187876, 146.01072348},
//  { 71.97919583, 142.99482022}},
// {{ 74.93151925, 139.98515158},
//  { 79.28391718, 141.27439129},
//  { 80.96689478, 140.45277677},
//  { 81.64809051, 139.06234624},
//  { 80.81320611, 137.61527327},
//  { 78.64729538, 135.47423465},
//  { 76.62224241, 131.15452694},
//  { 76.10018793, 122.98724413},
//  { 75.95978172, 107.95420622},
//  { 77.23102087, 100.10266295},
//  { 78.47215892,  93.91696187},
//  { 79.45325537,  87.00752589},
//  { 80.8292914 ,  77.00546255},
//  { 82.21922584,  71.11634457},
//  { 83.75942416,  66.86517951},
//  { 86.91234273,  60.96890362},
//  { 89.08599681,  57.99786435},
//  { 94.22786349,  52.16433773},
//  { 99.695333  ,  45.80738189},
//  {102.14095215,  42.08722396}},
// {{111.06272623,  33.09730436},
//  {114.82923666,  30.73492208},
//  {120.18257509,  26.29024672},
//  {123.05506798,  24.06082487},
//  {128.65914841,  20.57544985},
//  {132.22991539,  18.21613935},
//  {143.00293272,  12.15874365},
//  {146.96053675,  10.90456381},
//  {156.03181197,   9.88925189},
//  {175.94244173,  10.27517448},
//  {179.98998997,  11.06343283},
//  {187.1437228 ,  13.20089975},
//  {190.96998753,  14.33345071},
//  {194.91236679,  15.36359408},
//  {199.01797748,  16.93135548},
//  {210.03817756,  22.41148667},
//  {212.93726751,  23.57390439},
//  {222.10101843,  26.9175167 },
//  {227.90709287,  29.99042923},
//  {233.02600612,  35.0113091 }},
// {{235.02466789,  39.98751847},
//  {238.85623314,  44.04443372},
//  {240.17386855,  47.00947687},
//  {241.94317428,  49.9593286 },
//  {244.01991771,  51.65150655},
//  {245.07972092,  52.37603403},
//  {247.8236356 ,  55.06445218},
//  {248.6947607 ,  57.04350914},
//  {249.74176184,  60.69520389},
//  {253.44736805,  67.28557638},
//  {258.14229712,  74.85717282},
//  {259.17657509,  77.92261598},
//  {259.82995205,  81.12326378},
//  {261.17228456,  90.97694568},
//  {261.95836993,  94.0098842 },
//  {263.19097048,  96.82469865},
//  {264.52048895,  99.24493485},
//  {265.29648141, 105.89114825},
//  {266.90450889, 111.03322123},
//  {267.00296286, 125.99907476}},
// {{265.73828893, 131.95414344},
//  {265.57341995, 136.19291966},
//  {263.9535813 , 140.85034705},
//  {262.83508456, 142.90784723},
//  {261.59067905, 144.91274412},
//  {258.24282579, 150.15835832},
//  {257.16504408, 152.1924212 },
//  {255.8287282 , 161.59049004},
//  {257.07234814, 165.24072895}},
// {{266.34694124, 143.34791691},
//  {264.10089292, 141.27819499},
//  {264.04564225, 137.30021461},
//  {265.44461283, 132.13850405},
//  {266.8358681 , 125.99851327},
//  {267.41321649, 110.87736425},
//  {265.81282617, 103.05929191}},
// {{265.00387358,  98.00004541},
//  {261.9685935 ,  94.00208137},
//  {261.04550929,  90.99582055},
//  {259.93628347,  80.01035055},
//  {257.16384498,  71.9487227 },
//  {253.68150568,  66.14402658},
//  {250.26154084,  60.87333559},
//  {249.38735958,  58.79184763},
//  {248.61291145,  56.07690449},
//  {247.64660939,  54.30393154},
//  {246.00862193,  52.40635246},
//  {245.14749048,  51.52257746},
//  {242.37898319,  48.67955312},
//  {240.1011651 ,  46.02250555},
//  {238.32561504,  43.45267455},
//  {235.53950082,  38.6724237 },
//  {232.71199905,  34.09145543},
//  {229.03357972,  30.09096539},
//  {227.11562602,  28.86910526},
//  {224.92938688,  28.04532067}},
// {{215.88951249,  24.45779133},
//  {211.19896266,  22.17327546},
//  {197.89050884,  16.48699765},
//  {194.8817521 ,  15.41001932},
//  {191.00041094,  14.15354391},
//  {187.18600568,  13.05961703},
//  {179.98482388,  11.17060942},
//  {175.96802343,  10.08814588}},

//  };
//     // Initialize ROS
//     ros::init(argc, argv, "path_publisher");
//     ros::NodeHandle nh;

//     // Create a publisher for the optimized path
//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("optimized_path", 1);

//     // double pixelsPerMM = 2.834645590; // Adjust this value based on your image resolution
//     // double offsetX = 0.0;
//     // double offsetY = 0.0;
//     // std::vector<std::vector<Point>> convertedLines = convertToMM(lines, pixelsPerMM, offsetX, offsetY);

//     // Optimize the path with liftoff
//     nav_msgs::Path optimized_path = optimizePathWithLiftoff(lines);

//     // Write the optimized path to a CSV file
//     writeToCsv(optimized_path, "/home/darren2004/catkin_ws/src/optimize/src/optimized_path.csv");

//     // Publish the optimized path at a fixed rate
//     ros::Rate loop_rate(1);
//     while (ros::ok()) {
//         path_pub.publish(optimized_path);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }