/*
 * coeffs.h
 *
 *  Created on: Jul 21, 2020
 *      Author: xenir
 */

#ifndef INC_COEFFS_H_
#define INC_COEFFS_H_

#define NUM_TAPS	60

float band_pass_2_6[NUM_TAPS] =
{
		-23.19971815683497600E-6,
		-51.20262133777141860E-6,
		-4.838150854029422910E-12,
		 69.59374116305986040E-6,
		-49.95480922718883220E-6,
		-429.2551386426407590E-6,
		-657.6938147044506880E-6,
		-7.838064361734602410E-12,
		 0.001769362180628562,
		 0.003630252743103675,
		 0.003860962938270680,
		 0.001942031393846593,
		-3.917425739242253790E-12,
		 0.001520531654208547,
		 0.007467348106909672,
		 0.013439467444994090,
		 0.012024353126522539,
		 6.346483951115122760E-12,
		-0.016075779607590143,
		-0.023254630947711449,
		-0.014451588141285237,
		 829.4011459014016050E-6,
		 18.10681733217396070E-12,
		-0.033110262635498822,
		-0.087415511861842335,
		-0.123150931091434143,
		-0.097366494546677021,
		 29.80625951759512300E-12,
		 0.129138617193722854,
		 0.220477581370772446,
		 0.220477581370772446,
		 0.129138617193722854,
		 29.80625951759512300E-12,
		-0.097366494546677021,
		-0.123150931091434143,
		-0.087415511861842335,
		-0.033110262635498822,
		 18.10681733217396070E-12,
		 829.4011459014016050E-6,
		-0.014451588141285237,
		-0.023254630947711449,
		-0.016075779607590143,
		 6.346483951115122760E-12,
		 0.012024353126522539,
		 0.013439467444994090,
		 0.007467348106909672,
		 0.001520531654208547,
		-3.917425739242253790E-12,
		 0.001942031393846593,
		 0.003860962938270680,
		 0.003630252743103675,
		 0.001769362180628562,
		-7.838064361734602410E-12,
		-657.6938147044506880E-6,
		-429.2551386426407590E-6,
		-49.95480922718883220E-6,
		 69.59374116305986040E-6,
		-4.838150854029422910E-12,
		-51.20262133777141860E-6,
		-23.19971815683497600E-6


};

float low_pass_3khz[NUM_TAPS] =
{
		-96.37464579060073790E-6,
		-239.5922410224250710E-6,
		-236.3866644854411160E-6,
		-2.158076902079195220E-6,
		 510.2559562923468660E-6,
		 0.001286383953416845,
		 0.002234094331159058,
		 0.003181184355529571,
		 0.003890215348743426,
		 0.004091359454488967,
		 0.003530532804035990,
		 0.002026581946409395,
		-471.5498250157864960E-6,
		-0.003838285809652535,
		-0.007745892812315048,
		-0.011668195105453104,
		-0.014917825986515122,
		-0.016716148646595486,
		-0.016289357550435389,
		-0.012979146956684395,
		-0.006352643733446109,
		 0.003705193386918102,
		 0.016932376001823603,
		 0.032675798700162108,
		 0.049928231758077674,
		 0.067415089353529495,
		 0.083722175327715726,
		 0.097449098668476702,
		 0.107368611573056216,
		 0.112570535158906665,
		 0.112570535158906665,
		 0.107368611573056216,
		 0.097449098668476702,
		 0.083722175327715726,
		 0.067415089353529495,
		 0.049928231758077674,
		 0.032675798700162108,
		 0.016932376001823603,
		 0.003705193386918102,
		-0.006352643733446109,
		-0.012979146956684395,
		-0.016289357550435389,
		-0.016716148646595486,
		-0.014917825986515122,
		-0.011668195105453104,
		-0.007745892812315048,
		-0.003838285809652535,
		-471.5498250157864960E-6,
		 0.002026581946409395,
		 0.003530532804035990,
		 0.004091359454488967,
		 0.003890215348743426,
		 0.003181184355529571,
		 0.002234094331159058,
		 0.001286383953416845,
		 510.2559562923468660E-6,
		-2.158076902079195220E-6,
		-236.3866644854411160E-6,
		-239.5922410224250710E-6,
		-96.37464579060073790E-6
};


#endif /* INC_COEFFS_H_ */
