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
-0.000117396508726827,
-0.000140866265331063,
-0.000167340170912580,
-0.000177784715619282,
-0.000212953014153581,
-0.000240460752155653,
-0.000255746371733638,
-0.000302007271093451,
-0.000328735239613971,
-0.000355824083210999,
-0.000406553812245271,
-0.000437839464201006,
-0.000480017805332243,
-0.000525553260520807,
-0.000576683664528324,
-0.000625433027195963,
-0.000662538355317977,
-0.000753874456234381,
-0.000784968263201231,
-0.000828990569843188,
-0.000971764062075677,
-0.000952762463253099,
-0.001043246825027674,
-0.001221830843512046,
-0.001133038863986617,
-0.001322610045102196,
-0.001486652773079057,
-0.001347024322136104,
-0.001670635460293799,
-0.001751828588445406,
-0.001630493685405341,
-0.002067073392630867,
-0.002025344468025006,
-0.002016938346298464,
-0.002471133387071996,
-0.002354245996157317,
-0.002509179628648654,
-0.002845943084731894,
-0.002823610825936553,
-0.003052816597903792,
-0.003202057805793859,
-0.003525565367929813,
-0.003532489013821142,
-0.003643659553117639,
-0.004498456813755092,
-0.003810244224521333,
-0.004389720630862533,
-0.005655050755152956,
-0.003811263430996961,
-0.005742053839842442,
-0.006735405524072562,
-0.003639141521135673,
-0.007987494282050282,
-0.007324702030169754,
-0.003680830098671757,
-0.011249698047675888,
-0.006962332581449722,
-0.004653615821859017,
-0.015336245844742932,
-0.005338475890659329,
-0.007564345017849373,
-0.019643277629892379,
-0.002542166653175660,
-0.013600146845112250,
-0.023168801918093531,
 0.000683614566763868,
-0.024057656695998748,
-0.024639062260329230,
 0.002660761636598165,
-0.040602942540333961,
-0.022656189596816329,
 0.000149844580756745,
-0.066840025911667017,
-0.015503285242846776,
-0.014576646247399239,
-0.116861787001987544,
 0.001718984236618228,
-0.079619964411920388,
-0.326113374966197356,
 0.151538463918625604,
 0.655095300391330348,
 0.265291509751005738,
-0.035717427595246182,
 0.123882810997384046,
 0.061097448057588343,
-0.008051978609447102,
 0.075620076648891407,
 0.020180681960197944,
 0.005189272518425751,
 0.051626211875252126,
 0.005360421979006033,
 0.012962094310914484,
 0.034948354261022614,
 0.000553657289665871,
 0.016668575895469304,
 0.022645690376208325,
 0.000412270395189603,
 0.017176383109651670,
 0.013985831686858732,
 0.002145886853773697,
 0.015493013884066922,
 0.008479495040162339,
 0.004101573138401639,
 0.012689174017079120,
 0.005467271488346154,
 0.005415129305885871,
 0.009691149795706186,
 0.004148712903657022,
 0.005825324770527806,
 0.007121414256428620,
 0.003744438003907127,
 0.005472192082070302,
 0.005250888981752762,
 0.003648534498998132,
 0.004681575125419818,
 0.004059026905846684,
 0.003503447589946800,
 0.003783071769278771,
 0.003356953784012161,
 0.003185719690761791,
 0.002999676978571475,
 0.002916672889951528,
 0.002730068780469126,
 0.002417823543658834,
 0.002562251365733855,
 0.002235443065895073,
 0.002017965602101198,
 0.002205396050933835,
 0.001790818725815139,
 0.001731340585063619,
 0.001833731221704166,
 0.001439287021537174,
 0.001490989537737772,
 0.001475020568400428,
 0.001178141816117625,
 0.001259112740438938,
 0.001161486738148653,
 0.000979435500814612,
 0.001029299092112208,
 0.000909099544895944,
 0.000813141018978095,
 0.000813403054269198,
 0.000714430033426124,
 0.000661162719323564,
 0.000625706796935898,
 0.000562784973062575,
 0.000519451110390908,
 0.000473090960558274,
 0.000438687615479881,
 0.000392277388438908,
 0.000353455050816006,
 0.000332365710868006,
 0.000284913633564752,
 0.000259602091852974,
 0.000240578682262835,
 0.000199183648970980,
 0.000184165489413906,
 0.000163766946482777,
 0.000132985603876476,
 0.000122463451235539
};

float plus45Coeffs[NUM_TAPS] =
{
 0.000122463451235096,
 0.000132985603876276,
 0.000163766946483253,
 0.000184165489413513,
 0.000199183648971228,
 0.000240578682263990,
 0.000259602091851788,
 0.000284913633563888,
 0.000332365710869219,
 0.000353455050815068,
 0.000392277388439323,
 0.000438687615483330,
 0.000473090960555783,
 0.000519451110385203,
 0.000562784973064719,
 0.000625706796942962,
 0.000661162719324556,
 0.000714430033418391,
 0.000813403054265497,
 0.000813141018985449,
 0.000909099544898711,
 0.001029299092107732,
 0.000979435500815960,
 0.001161486738148148,
 0.001259112740434436,
 0.001178141816121635,
 0.001475020568403334,
 0.001490989537733999,
 0.001439287021538563,
 0.001833731221704645,
 0.001731340585059680,
 0.001790818725817749,
 0.002205396050936862,
 0.002017965602098589,
 0.002235443065895431,
 0.002562251365734570,
 0.002417823543655606,
 0.002730068780469687,
 0.002916672889954453,
 0.002999676978570192,
 0.003185719690761102,
 0.003356953784013339,
 0.003783071769277013,
 0.003503447589946033,
 0.004059026905849378,
 0.004681575125420221,
 0.003648534498996426,
 0.005250888981753347,
 0.005472192082070166,
 0.003744438003904411,
 0.007121414256430046,
 0.005825324770530440,
 0.004148712903653559,
 0.009691149795706773,
 0.005415129305888692,
 0.005467271488340789,
 0.012689174017081396,
 0.004101573138406461,
 0.008479495040155367,
 0.015493013884070319,
 0.002145886853776626,
 0.013985831686848995,
 0.017176383109658494,
 0.000412270395190820,
 0.022645690376198506,
 0.016668575895480407,
 0.000553657289661067,
 0.034948354261013968,
 0.012962094310930346,
 0.005360421978992631,
 0.051626211875249670,
 0.005189272518445087,
 0.020180681960169221,
 0.075620076648900886,
-0.008051978609427789,
 0.061097448057531943,
 0.123882810997423209,
-0.035717427595234670,
 0.265291509750805066,
 0.655095300391341673,
 0.151538463918894445,
-0.326113374966181868,
-0.079619964412029801,
 0.001718984236664280,
-0.116861787001986864,
-0.014576646247436983,
-0.015503285242816739,
-0.066840025911677661,
 0.000149844580741854,
-0.022656189596796508,
-0.040602942540347485,
 0.002660761636596170,
-0.024639062260317091,
-0.024057656696011912,
 0.000683614566767649,
-0.023168801918088084,
-0.013600146845122792,
-0.002542166653168626,
-0.019643277629890758,
-0.007564345017857459,
-0.005338475890653293,
-0.015336245844744462,
-0.004653615821864418,
-0.006962332581443665,
-0.011249698047676686,
-0.003680830098674958,
-0.007324702030166080,
-0.007987494282051753,
-0.003639141521138173,
-0.006735405524069521,
-0.005742053839841739,
-0.003811263430998693,
-0.005655050755152268,
-0.004389720630862675,
-0.003810244224523792,
-0.004498456813754547,
-0.003643659553115191,
-0.003532489013822136,
-0.003525565367930459,
-0.003202057805792551,
-0.003052816597905739,
-0.002823610825937566,
-0.002845943084728450,
-0.002509179628648645,
-0.002354245996159436,
-0.002471133387070325,
-0.002016938346299982,
-0.002025344468028046,
-0.002067073392626976,
-0.001630493685404010,
-0.001751828588448484,
-0.001670635460291455,
-0.001347024322136893,
-0.001486652773083443,
-0.001322610045097884,
-0.001133038863983961,
-0.001221830843515427,
-0.001043246825025623,
-0.000952762463254016,
-0.000971764062081976,
-0.000828990569839427,
-0.000784968263193475,
-0.000753874456237041,
-0.000662538355325449,
-0.000625433027196744,
-0.000576683664521828,
-0.000525553260518064,
-0.000480017805336911,
-0.000437839464202587,
-0.000406553812243105,
-0.000355824083211598,
-0.000328735239613855,
-0.000302007271092069,
-0.000255746371734848,
-0.000240460752156411,
-0.000212953014152760,
-0.000177784715619481,
-0.000167340170912605,
-0.000140866265330519,
-0.000117396508727112
};


float high_pass_2khz[NUM_TAPS] =
{
		-708.0851878245794070E-9,
		-1.187750167406475650E-6,
		-1.254152829530071010E-6,
		-824.3554660230807940E-9,
		 40.52186914664007130E-9,
		 1.127045074690356200E-6,
		 2.102912442799850900E-6,
		 2.590579613318962690E-6,
		 2.269201355725880110E-6,
		 983.9715698144941600E-9,
		-1.163529515922776980E-6,
		-3.770453703262529820E-6,
		-6.153592436677942420E-6,
		-7.444461120111261820E-6,
		-6.751030381826908840E-6,
		-3.361639130830817560E-6,
		 3.047691787687178520E-6,
		 12.24940391647061590E-6,
		 23.38988410459466390E-6,
		 35.03884060381788150E-6,
		 45.37618590319409860E-6,
		 52.50239162845977600E-6,
		 54.82872275328547570E-6,
		 51.47825995395254490E-6,
		 42.61326207438304440E-6,
		 29.60416126966546100E-6,
		 14.97296152329266940E-6,
		 2.078457491588369790E-6,
		-5.441656256930757610E-6,
		-4.403531832517408870E-6,
		 7.132381626343135220E-6,
		 29.16437072298856490E-6,
		 59.30695050784611060E-6,
		 92.74317248343722040E-6,
		 122.6598157944781060E-6,
		 141.1742440153718120E-6,
		 140.6660578228174930E-6,
		 115.3301753557504840E-6,
		 62.68889488869648120E-6,
		-15.24310805319540310E-6,
		-111.4405410540128680E-6,
		-214.2316707178407000E-6,
		-308.6603450026912010E-6,
		-378.7967129536058340E-6,
		-410.7602476007301110E-6,
		-396.0380009753908440E-6,
		-334.5397692709894390E-6,
		-236.7614201596480830E-6,
		-124.4524486833755500E-6,
		-29.31622339532786730E-6,
		 10.49213405964626840E-6,
		-44.01429872773717730E-6,
		-225.3662234130297580E-6,
		-551.4924459088447290E-6,
		-0.001017826334840902,
		-0.001590957098654717,
		-0.002205140024595489,
		-0.002762813220005102,
		-0.003139911691332094,
		-0.003196248536647381,
		-0.002790600586324826,
		-0.001799467640556280,
		-137.8606564258881800E-6,
		 0.002219991271484400,
		 0.005222356549215390,
		 0.008726410253726360,
		 0.012491193198613429,
		 0.016178201101485646,
		 0.019359540458149326,
		 0.021532046283435972,
		 0.022133677051654109,
		 0.020555426776850998,
		 0.016136605655480904,
		 0.008119861435757139,
		-0.004486834093921676,
		-0.023280924136609833,
		-0.051519362292753793,
		-0.097739400446096261,
		-0.193864553337536050,
		-0.630404933479642216,
		 0.630404933479642216,
		 0.193864553337536050,
		 0.097739400446096261,
		 0.051519362292753793,
		 0.023280924136609833,
		 0.004486834093921676,
		-0.008119861435757139,
		-0.016136605655480904,
		-0.020555426776850998,
		-0.022133677051654109,
		-0.021532046283435972,
		-0.019359540458149326,
		-0.016178201101485646,
		-0.012491193198613429,
		-0.008726410253726360,
		-0.005222356549215390,
		-0.002219991271484400,
		 137.8606564258881800E-6,
		 0.001799467640556280,
		 0.002790600586324826,
		 0.003196248536647381,
		 0.003139911691332094,
		 0.002762813220005102,
		 0.002205140024595489,
		 0.001590957098654717,
		 0.001017826334840902,
		 551.4924459088447290E-6,
		 225.3662234130297580E-6,
		 44.01429872773717730E-6,
		-10.49213405964626840E-6,
		 29.31622339532786730E-6,
		 124.4524486833755500E-6,
		 236.7614201596480830E-6,
		 334.5397692709894390E-6,
		 396.0380009753908440E-6,
		 410.7602476007301110E-6,
		 378.7967129536058340E-6,
		 308.6603450026912010E-6,
		 214.2316707178407000E-6,
		 111.4405410540128680E-6,
		 15.24310805319540310E-6,
		-62.68889488869648120E-6,
		-115.3301753557504840E-6,
		-140.6660578228174930E-6,
		-141.1742440153718120E-6,
		-122.6598157944781060E-6,
		-92.74317248343722040E-6,
		-59.30695050784611060E-6,
		-29.16437072298856490E-6,
		-7.132381626343135220E-6,
		 4.403531832517408870E-6,
		 5.441656256930757610E-6,
		-2.078457491588369790E-6,
		-14.97296152329266940E-6,
		-29.60416126966546100E-6,
		-42.61326207438304440E-6,
		-51.47825995395254490E-6,
		-54.82872275328547570E-6,
		-52.50239162845977600E-6,
		-45.37618590319409860E-6,
		-35.03884060381788150E-6,
		-23.38988410459466390E-6,
		-12.24940391647061590E-6,
		-3.047691787687178520E-6,
		 3.361639130830817560E-6,
		 6.751030381826908840E-6,
		 7.444461120111261820E-6,
		 6.153592436677942420E-6,
		 3.770453703262529820E-6,
		 1.163529515922776980E-6,
		-983.9715698144941600E-9,
		-2.269201355725880110E-6,
		-2.590579613318962690E-6,
		-2.102912442799850900E-6,
		-1.127045074690356200E-6,
		-40.52186914664007130E-9,
		 824.3554660230807940E-9,
		 1.254152829530071010E-6,
		 1.187750167406475650E-6,
		 708.0851878245794070E-9


};

float low_pass_2khz[NUM_TAPS] =
{
		215.8106272749660660E-9,
		 115.3504966445241000E-9,
		-324.7388173760813860E-9,
		-1.047571572314596630E-6,
		-1.914875505604531460E-6,
		-2.722208854542560450E-6,
		-3.230205533776336240E-6,
		-3.208617820943161720E-6,
		-2.487180376941221290E-6,
		-1.005200947412471460E-6,
		 1.149276456618246290E-6,
		 3.719572609861942600E-6,
		 6.286739662435913090E-6,
		 8.305921464068234170E-6,
		 9.172041770071597710E-6,
		 8.309174436611561630E-6,
		 5.273083419884610380E-6,
		-147.5521699435730060E-9,
		-7.848038237788673930E-6,
		-17.35956955430992470E-6,
		-27.85046921258374650E-6,
		-38.18591692559017760E-6,
		-47.04787985571486790E-6,
		-53.10801844532800690E-6,
		-55.23698747469278200E-6,
		-52.72510077219245520E-6,
		-45.48315283664113910E-6,
		-34.18954770094905630E-6,
		-20.35171704155973770E-6,
		-6.256570438437140780E-6,
		 5.203752905478134850E-6,
		 10.82911564950770790E-6,
		 7.512024590461497730E-6,
		-7.296095682216627410E-6,
		-35.10691747232910840E-6,
		-75.97100297479104820E-6,
		-128.1775895891032630E-6,
		-188.1447348123691370E-6,
		-250.5528923039366870E-6,
		-308.7524390685252910E-6,
		-355.4452488510654580E-6,
		-383.6046674551123490E-6,
		-387.5607669751354930E-6,
		-364.1428281589276140E-6,
		-313.7432063460399830E-6,
		-241.1504469021603820E-6,
		-155.9983506028494840E-6,
		-72.69400762954168730E-6,
		-9.722300353386541970E-6,
		 11.72425296199059380E-6,
		-30.22351580746502720E-6,
		-155.5067832189803600E-6,
		-379.1226190966432340E-6,
		-707.9275635374344800E-6,
		-0.001137547104491560,
		-0.001649711031784082,
		-0.002210341217247015,
		-0.002768693997384386,
		-0.003257803011828333,
		-0.003596382170398337,
		-0.003692237622671311,
		-0.003447110302025585,
		-0.002762737219864461,
		-0.001547792040470864,
		 274.7442041078185180E-6,
		 0.002760311659581526,
		 0.005936170210227700,
		 0.009796034739109000,
		 0.014296461526277886,
		 0.019355401292845274,
		 0.024853194591897030,
		 0.030636114359171851,
		 0.036522371517719449,
		 0.042310307816839553,
		 0.047788321678996007,
		 0.052745923254801833,
		 0.056985207661009264,
		 0.060331980721193917,
		 0.062645775364681425,
		 0.063828060160748504,
		 0.063828060160748504,
		 0.062645775364681425,
		 0.060331980721193917,
		 0.056985207661009264,
		 0.052745923254801833,
		 0.047788321678996007,
		 0.042310307816839553,
		 0.036522371517719449,
		 0.030636114359171851,
		 0.024853194591897030,
		 0.019355401292845274,
		 0.014296461526277886,
		 0.009796034739109000,
		 0.005936170210227700,
		 0.002760311659581526,
		 274.7442041078185180E-6,
		-0.001547792040470864,
		-0.002762737219864461,
		-0.003447110302025585,
		-0.003692237622671311,
		-0.003596382170398337,
		-0.003257803011828333,
		-0.002768693997384386,
		-0.002210341217247015,
		-0.001649711031784082,
		-0.001137547104491560,
		-707.9275635374344800E-6,
		-379.1226190966432340E-6,
		-155.5067832189803600E-6,
		-30.22351580746502720E-6,
		 11.72425296199059380E-6,
		-9.722300353386541970E-6,
		-72.69400762954168730E-6,
		-155.9983506028494840E-6,
		-241.1504469021603820E-6,
		-313.7432063460399830E-6,
		-364.1428281589276140E-6,
		-387.5607669751354930E-6,
		-383.6046674551123490E-6,
		-355.4452488510654580E-6,
		-308.7524390685252910E-6,
		-250.5528923039366870E-6,
		-188.1447348123691370E-6,
		-128.1775895891032630E-6,
		-75.97100297479104820E-6,
		-35.10691747232910840E-6,
		-7.296095682216627410E-6,
		 7.512024590461497730E-6,
		 10.82911564950770790E-6,
		 5.203752905478134850E-6,
		-6.256570438437140780E-6,
		-20.35171704155973770E-6,
		-34.18954770094905630E-6,
		-45.48315283664113910E-6,
		-52.72510077219245520E-6,
		-55.23698747469278200E-6,
		-53.10801844532800690E-6,
		-47.04787985571486790E-6,
		-38.18591692559017760E-6,
		-27.85046921258374650E-6,
		-17.35956955430992470E-6,
		-7.848038237788673930E-6,
		-147.5521699435730060E-9,
		 5.273083419884610380E-6,
		 8.309174436611561630E-6,
		 9.172041770071597710E-6,
		 8.305921464068234170E-6,
		 6.286739662435913090E-6,
		 3.719572609861942600E-6,
		 1.149276456618246290E-6,
		-1.005200947412471460E-6,
		-2.487180376941221290E-6,
		-3.208617820943161720E-6,
		-3.230205533776336240E-6,
		-2.722208854542560450E-6,
		-1.914875505604531460E-6,
		-1.047571572314596630E-6,
		-324.7388173760813860E-9,
		 115.3504966445241000E-9,
		 215.8106272749660660E-9
};


#endif /* INC_COEFFS_H_ */
