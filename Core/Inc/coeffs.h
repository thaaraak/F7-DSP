/*
 * coeffs.h
 *
 *  Created on: Jul 21, 2020
 *      Author: xenir
 */

#ifndef INC_COEFFS_H_
#define INC_COEFFS_H_

#define NUM_TAPS	160

float minus45Coeffs[NUM_TAPS] =
{
		-0.000120287953900828,
		-0.000135945428333644,
		-0.000155682453860058,
		-0.000181517359655903,
		-0.000212353658151604,
		-0.000244235342471218,
		-0.000273169442694727,
		-0.000298510258329736,
		-0.000323959013787682,
		-0.000354877578562898,
		-0.000393829595470117,
		-0.000438281941129542,
		-0.000483008249152025,
		-0.000525558539504562,
		-0.000569702527692960,
		-0.000622641204775897,
		-0.000687208092400690,
		-0.000755973872939229,
		-0.000814451114978083,
		-0.000853605630211834,
		-0.000882406169332198,
		-0.000927770878620730,
		-0.001017135540943839,
		-0.001153821996787365,
		-0.001305409560861023,
		-0.001419691307057085,
		-0.001462372491215726,
		-0.001449982621404223,
		-0.001448913417299795,
		-0.001533644949895902,
		-0.001730276611698551,
		-0.001989021163003184,
		-0.002213011659853736,
		-0.002328197830020478,
		-0.002342275074262780,
		-0.002342471336868994,
		-0.002427622437049129,
		-0.002626576740817701,
		-0.002874086636007348,
		-0.003073085968544271,
		-0.003195771280267560,
		-0.003329222327067972,
		-0.003604747829268983,
		-0.004051106055204852,
		-0.004501412146959981,
		-0.004673148973195809,
		-0.004410646808545734,
		-0.003915679504893039,
		-0.003736693392773420,
		-0.004426414535107496,
		-0.006047742848029463,
		-0.007909911672298512,
		-0.008851617438135955,
		-0.008027666248453288,
		-0.005716309070823912,
		-0.003490251581960202,
		-0.003407121860732349,
		-0.006559509471850598,
		-0.011933818133067696,
		-0.016556941683301133,
		-0.017183742610413607,
		-0.012665207954393564,
		-0.005393724448905768,
		-0.000502273867338047,
		-0.002831583865235728,
		-0.013326057684912655,
		-0.027337452054551171,
		-0.036529654681449489,
		-0.033908877662463317,
		-0.019197934776996252,
		-0.000910348751381076,
		 0.007071619194155779,
		-0.006591696458091912,
		-0.042434180281795457,
		-0.086708229149332877,
		-0.115747930961879852,
		-0.106942120320850056,
		-0.051002209483948353,
		 0.041170736598053877,
		 0.141315683268417286,
		 0.215588329209099122,
		 0.239626094727750649,
		 0.209297341592480202,
		 0.141728302832795749,
		 0.066339606773771459,
		 0.010812511620459733,
		-0.010734135473486289,
		-0.001730920482761196,
		 0.021776527573613935,
		 0.041339777865803014,
		 0.045650990515360063,
		 0.034627882112233270,
		 0.016853005284858488,
		 0.002934377977481613,
		-0.000856551676058310,
		 0.004776574735072420,
		 0.013990341524699508,
		 0.020195296111494974,
		 0.019988210621937220,
		 0.014441766296456954,
		 0.007528557310705374,
		 0.003181255658327593,
		 0.002999284326015943,
		 0.005824036583837862,
		 0.009038359733127208,
		 0.010440876922640254,
		 0.009443781746175023,
		 0.007025597558979157,
		 0.004756188087883347,
		 0.003707824647709193,
		 0.003946324051996303,
		 0.004769869411196012,
		 0.005354060702195154,
		 0.005291130910762627,
		 0.004710704324673022,
		 0.004022094651339535,
		 0.003555372091981304,
		 0.003370190093078619,
		 0.003313081240621682,
		 0.003208717355365067,
		 0.003006226249561500,
		 0.002782263101012495,
		 0.002635805360830143,
		 0.002585630721262003,
		 0.002555832305600688,
		 0.002447801501375700,
		 0.002227212265594207,
		 0.001952467707523887,
		 0.001727067290121745,
		 0.001620142040478467,
		 0.001618793251440919,
		 0.001644340148280096,
		 0.001613257381933391,
		 0.001493599392225158,
		 0.001317980205369419,
		 0.001150955066325084,
		 0.001040994500218117,
		 0.000992606711008434,
		 0.000973462750115447,
		 0.000944198079518095,
		 0.000885528029635116,
		 0.000804880420376507,
		 0.000723214877646191,
		 0.000656459769620356,
		 0.000606158152823550,
		 0.000563450022931608,
		 0.000519592191568655,
		 0.000473082719444775,
		 0.000428665250366757,
		 0.000391117996592757,
		 0.000360296619040867,
		 0.000331476978922465,
		 0.000299823557747422,
		 0.000264551482466095,
		 0.000229310814327694,
		 0.000198950919190529,
		 0.000175722285918519,
		 0.000157949487003571,
		 0.000141794952908107,
		 0.000124259469056090

};

float plus45Coeffs[NUM_TAPS] =
{
		 0.000124259469056082,
		 0.000141794952908108,
		 0.000157949487003583,
		 0.000175722285918541,
		 0.000198950919190558,
		 0.000229310814327723,
		 0.000264551482466118,
		 0.000299823557747431,
		 0.000331476978922455,
		 0.000360296619040837,
		 0.000391117996592711,
		 0.000428665250366704,
		 0.000473082719444729,
		 0.000519592191568629,
		 0.000563450022931609,
		 0.000606158152823578,
		 0.000656459769620401,
		 0.000723214877646240,
		 0.000804880420376549,
		 0.000885528029635142,
		 0.000944198079518098,
		 0.000973462750115419,
		 0.000992606711008370,
		 0.001040994500218017,
		 0.001150955066324964,
		 0.001317980205369313,
		 0.001493599392225105,
		 0.001613257381933417,
		 0.001644340148280199,
		 0.001618793251441061,
		 0.001620142040478595,
		 0.001727067290121817,
		 0.001952467707523889,
		 0.002227212265594157,
		 0.002447801501375624,
		 0.002555832305600598,
		 0.002585630721261892,
		 0.002635805360830011,
		 0.002782263101012360,
		 0.003006226249561415,
		 0.003208717355365077,
		 0.003313081240621788,
		 0.003370190093078750,
		 0.003555372091981361,
		 0.004022094651339461,
		 0.004710704324672859,
		 0.005291130910762506,
		 0.005354060702195192,
		 0.004769869411196179,
		 0.003946324051996394,
		 0.003707824647708943,
		 0.004756188087882659,
		 0.007025597558978277,
		 0.009443781746174473,
		 0.010440876922640537,
		 0.009038359733128378,
		 0.005824036583839337,
		 0.002999284326016725,
		 0.003181255658326886,
		 0.007528557310703224,
		 0.014441766296454441,
		 0.019988210621935936,
		 0.020195296111495998,
		 0.013990341524702453,
		 0.004776574735075384,
		-0.000856551676057765,
		 0.002934377977478398,
		 0.016853005284852521,
		 0.034627882112227802,
		 0.045650990515358848,
		 0.041339777865807802,
		 0.021776527573622615,
		-0.001730920482754359,
		-0.010734135473488013,
		 0.010812511620445562,
		 0.066339606773746770,
		 0.141728302832768466,
		 0.209297341592461356,
		 0.239626094727749234,
		 0.215588329209117635,
		 0.141315683268450315,
		 0.041170736598090223,
		-0.051002209483920577,
		-0.106942120320838344,
		-0.115747930961884293,
		-0.086708229149347324,
		-0.042434180281810931,
		-0.006591696458101262,
		 0.007071619194154991,
		-0.000910348751375718,
		-0.019197934776989691,
		-0.033908877662459876,
		-0.036529654681450696,
		-0.027337452054555539,
		-0.013326057684917188,
		-0.002831583865237937,
		-0.000502273867337293,
		-0.005393724448903255,
		-0.012665207954391270,
		-0.017183742610412969,
		-0.016556941683302337,
		-0.011933818133069801,
		-0.006559509471852332,
		-0.003407121860732907,
		-0.003490251581959609,
		-0.005716309070822810,
		-0.008027666248452425,
		-0.008851617438135728,
		-0.007909911672298845,
		-0.006047742848029985,
		-0.004426414535107865,
		-0.003736693392773517,
		-0.003915679504892971,
		-0.004410646808545685,
		-0.004673148973195890,
		-0.004501412146960157,
		-0.004051106055205017,
		-0.003604747829269037,
		-0.003329222327067903,
		-0.003195771280267432,
		-0.003073085968544160,
		-0.002874086636007301,
		-0.002626576740817716,
		-0.002427622437049180,
		-0.002342471336869061,
		-0.002342275074262868,
		-0.002328197830020595,
		-0.002213011659853873,
		-0.001989021163003308,
		-0.001730276611698617,
		-0.001533644949895882,
		-0.001448913417299697,
		-0.001449982621404085,
		-0.001462372491215600,
		-0.001419691307057008,
		-0.001305409560861010,
		-0.001153821996787404,
		-0.001017135540943906,
		-0.000927770878620804,
		-0.000882406169332265,
		-0.000853605630211886,
		-0.000814451114978115,
		-0.000755973872939236,
		-0.000687208092400671,
		-0.000622641204775856,
		-0.000569702527692909,
		-0.000525558539504516,
		-0.000483008249151997,
		-0.000438281941129540,
		-0.000393829595470138,
		-0.000354877578562935,
		-0.000323959013787725,
		-0.000298510258329774,
		-0.000273169442694755,
		-0.000244235342471231,
		-0.000212353658151602,
		-0.000181517359655889,
		-0.000155682453860037,
		-0.000135945428333623,
		-0.000120287953900810

};


float high_pass_2khz[NUM_TAPS] =
{
		-58.88294163331768520E-6,
		-50.56229561949692910E-6,
		-37.28036876850829630E-6,
		-21.00837217451814710E-6,
		-4.352441711128783200E-6,
		 9.883641681231624790E-6,
		 19.20752208794547040E-6,
		 21.93131919977428270E-6,
		 17.55655673611547840E-6,
		 6.970939688218090070E-6,
		-7.595455109551521030E-6,
		-22.86759007468855030E-6,
		-35.04435397585064040E-6,
		-40.46357312481844560E-6,
		-36.30441319369479200E-6,
		-21.20174626755860460E-6,
		 4.348480890225569160E-6,
		 37.88713682924453960E-6,
		 75.24113059147823890E-6,
		 111.1128509478419490E-6,
		 139.9358343284296210E-6,
		 156.8670022853849840E-6,
		 158.7479214174637720E-6,
		 144.8558955197732980E-6,
		 117.2849588740300110E-6,
		 80.84669120824096920E-6,
		 42.45552505748192830E-6,
		 10.05226218419061010E-6,
		-8.791302342846668340E-6,
		-8.370063845682880200E-6,
		 13.81460552237222570E-6,
		 56.13581310281280420E-6,
		 112.5623160152191820E-6,
		 173.1522892100230420E-6,
		 225.3312824711523210E-6,
		 255.8282978700399040E-6,
		 253.0109706829935020E-6,
		 209.2558348641511540E-6,
		 122.9338293889172750E-6,
		-399.9238132874105530E-9,
		-147.9428086223484230E-6,
		-300.8912780516428710E-6,
		-436.9819550751693670E-6,
		-534.0876409735067230E-6,
		-574.4118312581248350E-6,
		-548.6519610762256890E-6,
		-459.4049994727200210E-6,
		-323.0946521356989360E-6,
		-169.8201012487945430E-6,
		-40.75870156476134550E-6,
		 16.92107200435424020E-6,
		-42.69867975430821620E-6,
		-255.6090156902834560E-6,
		-638.8192764379568870E-6,
		-0.001182211947384658,
		-0.001842678869406329,
		-0.002541814918720051,
		-0.003168163245601912,
		-0.003584554730962581,
		-0.003640505584748057,
		-0.003188994624207739,
		-0.002106314438436681,
		-313.1632468389738050E-6,
		 0.002205202221515002,
		 0.005382030696289503,
		 0.009057438778713355,
		 0.012973640919989694,
		 0.016777786231410762,
		 0.020031997733453988,
		 0.022228722563537896,
		 0.022807522487106068,
		 0.021166489809738582,
		 0.016656225941943846,
		 0.008532968070865645,
		-0.004181604901677204,
		-0.023073226901063903,
		-0.051390852273327514,
		-0.097668481729352746,
		-0.193831331509850935,
		-0.630395517417273732,
		 0.630395517417273732,
		 0.193831331509850935,
		 0.097668481729352746,
		 0.051390852273327514,
		 0.023073226901063903,
		 0.004181604901677204,
		-0.008532968070865645,
		-0.016656225941943846,
		-0.021166489809738582,
		-0.022807522487106068,
		-0.022228722563537896,
		-0.020031997733453988,
		-0.016777786231410762,
		-0.012973640919989694,
		-0.009057438778713355,
		-0.005382030696289503,
		-0.002205202221515002,
		 313.1632468389738050E-6,
		 0.002106314438436681,
		 0.003188994624207739,
		 0.003640505584748057,
		 0.003584554730962581,
		 0.003168163245601912,
		 0.002541814918720051,
		 0.001842678869406329,
		 0.001182211947384658,
		 638.8192764379568870E-6,
		 255.6090156902834560E-6,
		 42.69867975430821620E-6,
		-16.92107200435424020E-6,
		 40.75870156476134550E-6,
		 169.8201012487945430E-6,
		 323.0946521356989360E-6,
		 459.4049994727200210E-6,
		 548.6519610762256890E-6,
		 574.4118312581248350E-6,
		 534.0876409735067230E-6,
		 436.9819550751693670E-6,
		 300.8912780516428710E-6,
		 147.9428086223484230E-6,
		 399.9238132874105530E-9,
		-122.9338293889172750E-6,
		-209.2558348641511540E-6,
		-253.0109706829935020E-6,
		-255.8282978700399040E-6,
		-225.3312824711523210E-6,
		-173.1522892100230420E-6,
		-112.5623160152191820E-6,
		-56.13581310281280420E-6,
		-13.81460552237222570E-6,
		 8.370063845682880200E-6,
		 8.791302342846668340E-6,
		-10.05226218419061010E-6,
		-42.45552505748192830E-6,
		-80.84669120824096920E-6,
		-117.2849588740300110E-6,
		-144.8558955197732980E-6,
		-158.7479214174637720E-6,
		-156.8670022853849840E-6,
		-139.9358343284296210E-6,
		-111.1128509478419490E-6,
		-75.24113059147823890E-6,
		-37.88713682924453960E-6,
		-4.348480890225569160E-6,
		 21.20174626755860460E-6,
		 36.30441319369479200E-6,
		 40.46357312481844560E-6,
		 35.04435397585064040E-6,
		 22.86759007468855030E-6,
		 7.595455109551521030E-6,
		-6.970939688218090070E-6,
		-17.55655673611547840E-6,
		-21.93131919977428270E-6,
		-19.20752208794547040E-6,
		-9.883641681231624790E-6,
		 4.352441711128783200E-6,
		 21.00837217451814710E-6,
		 37.28036876850829630E-6,
		 50.56229561949692910E-6,
		 58.88294163331768520E-6

};

float low_pass_2khz[NUM_TAPS] =
{
		 6.257963695487910090E-6,
		-16.67040744386494570E-6,
		-37.12468714970370340E-6,
		-50.97888785810830110E-6,
		-55.59176994180462830E-6,
		-50.57046135081760950E-6,
		-37.99444215451306660E-6,
		-21.99824808227207380E-6,
		-7.772173259808462300E-6,
		-198.8613762226628410E-9,
		-2.458326622480390360E-6,
		-14.97101056283125690E-6,
		-34.99169444988951480E-6,
		-57.02319321610072220E-6,
		-74.01769701712989050E-6,
		-79.12277129372071730E-6,
		-67.56245832856073720E-6,
		-38.16907335445185370E-6,
		 5.873870039843915870E-6,
		 57.35306532244574380E-6,
		 106.4345417026315770E-6,
		 142.9008568405878350E-6,
		 158.7130946371397610E-6,
		 150.2573793165603890E-6,
		 119.6514584851978070E-6,
		 74.66724010779697100E-6,
		 27.14053081695562500E-6,
		-9.877542366691859140E-6,
		-25.61344855266566970E-6,
		-14.85409720933651560E-6,
		 19.88090155497967260E-6,
		 67.99581196186962020E-6,
		 112.6320691196891060E-6,
		 134.4431199495835760E-6,
		 116.5976356162282740E-6,
		 49.86042356444520610E-6,
		-63.57532696902811150E-6,
		-208.6893648214335140E-6,
		-359.3902432631272750E-6,
		-483.4348316723889520E-6,
		-549.7063636798063730E-6,
		-536.3192837873978080E-6,
		-437.5735372660721510E-6,
		-267.7668903513257420E-6,
		-60.35730047665218480E-6,
		 138.0970900885023130E-6,
		 278.5654483074797550E-6,
		 322.8178443809889590E-6,
		 256.4157062750553560E-6,
		 98.02064030927887470E-6,
		-98.11222129910392200E-6,
		-248.5366466649427370E-6,
		-256.2676940000510510E-6,
		-33.92372175628661070E-6,
		 469.4941318273995990E-6,
		 0.001243641471613832,
		 0.002199837614135980,
		 0.003167678806140006,
		 0.003909109357958581,
		 0.004150916834472744,
		 0.003633060909285564,
		 0.002166635136432881,
		-307.6991720845717850E-6,
		-0.003671617135530390,
		-0.007602038996185488,
		-0.011573580161274372,
		-0.014895125816970268,
		-0.016779853910365267,
		-0.016442273453553764,
		-0.013210562941762634,
		-0.006638655599972962,
		 0.003399086340277936,
		 0.016646682754167815,
		 0.032451118975072253,
		 0.049798946810551109,
		 0.067403715905775929,
		 0.083835367636273112,
		 0.097676033203338702,
		 0.107682098539451460,
		 0.112930739446503489,
		 0.112930739446503489,
		 0.107682098539451460,
		 0.097676033203338702,
		 0.083835367636273112,
		 0.067403715905775929,
		 0.049798946810551109,
		 0.032451118975072253,
		 0.016646682754167815,
		 0.003399086340277936,
		-0.006638655599972962,
		-0.013210562941762634,
		-0.016442273453553764,
		-0.016779853910365267,
		-0.014895125816970268,
		-0.011573580161274372,
		-0.007602038996185488,
		-0.003671617135530390,
		-307.6991720845717850E-6,
		 0.002166635136432881,
		 0.003633060909285564,
		 0.004150916834472744,
		 0.003909109357958581,
		 0.003167678806140006,
		 0.002199837614135980,
		 0.001243641471613832,
		 469.4941318273995990E-6,
		-33.92372175628661070E-6,
		-256.2676940000510510E-6,
		-248.5366466649427370E-6,
		-98.11222129910392200E-6,
		 98.02064030927887470E-6,
		 256.4157062750553560E-6,
		 322.8178443809889590E-6,
		 278.5654483074797550E-6,
		 138.0970900885023130E-6,
		-60.35730047665218480E-6,
		-267.7668903513257420E-6,
		-437.5735372660721510E-6,
		-536.3192837873978080E-6,
		-549.7063636798063730E-6,
		-483.4348316723889520E-6,
		-359.3902432631272750E-6,
		-208.6893648214335140E-6,
		-63.57532696902811150E-6,
		 49.86042356444520610E-6,
		 116.5976356162282740E-6,
		 134.4431199495835760E-6,
		 112.6320691196891060E-6,
		 67.99581196186962020E-6,
		 19.88090155497967260E-6,
		-14.85409720933651560E-6,
		-25.61344855266566970E-6,
		-9.877542366691859140E-6,
		 27.14053081695562500E-6,
		 74.66724010779697100E-6,
		 119.6514584851978070E-6,
		 150.2573793165603890E-6,
		 158.7130946371397610E-6,
		 142.9008568405878350E-6,
		 106.4345417026315770E-6,
		 57.35306532244574380E-6,
		 5.873870039843915870E-6,
		-38.16907335445185370E-6,
		-67.56245832856073720E-6,
		-79.12277129372071730E-6,
		-74.01769701712989050E-6,
		-57.02319321610072220E-6,
		-34.99169444988951480E-6,
		-14.97101056283125690E-6,
		-2.458326622480390360E-6,
		-198.8613762226628410E-9,
		-7.772173259808462300E-6,
		-21.99824808227207380E-6,
		-37.99444215451306660E-6,
		-50.57046135081760950E-6,
		-55.59176994180462830E-6,
		-50.97888785810830110E-6,
		-37.12468714970370340E-6,
		-16.67040744386494570E-6,
		 6.257963695487910090E-6

};


#endif /* INC_COEFFS_H_ */
