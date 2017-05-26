package org.usfirst.frc.team4028.robot.util;

import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;

public class TwoGearLong extends MotionProfile{
			
	public TwoGearLong() {
		super(1.0, 1.0, false, GearShiftPosition.LOW_GEAR);
		// TODO Auto-generated constructor stub
	}

	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.000825, 0.041251, 2.062509, 0.010000, 0.02},
		{0.003225, 0.120006, 3.937621, 0.010000, 0.02},
		{0.007226, 0.200016, 4.000431, 0.010000, 0.02},
		{0.012826, 0.280035, 4.000956, 0.009999, 0.02},
		{0.020027, 0.360066, 4.001695, 0.009998, 0.02},
		{0.028854, 0.440214, 3.997459, 0.009996, 0.02},
		{0.039254, 0.520360, 4.009794, 0.009993, 0.02},
		{0.051254, 0.600402, 4.005049, 0.009989, 0.02},
		{0.064849, 0.680459, 4.006729, 0.009982, 0.02},
		{0.080064, 0.760584, 4.005480, 0.009973, 0.02},
		{0.096894, 0.840816, 4.008425, 0.009960, 0.02},
		{0.115307, 0.921060, 4.013905, 0.009944, 0.02},
		{0.135322, 1.001294, 4.013823, 0.009923, 0.02},
		{0.156955, 1.081590, 4.014728, 0.009897, 0.02},
		{0.180191, 1.161952, 4.018460, 0.009865, 0.02},
		{0.205043, 1.242371, 4.020328, 0.009827, 0.02},
		{0.231516, 1.322874, 4.022820, 0.009780, 0.02},
		{0.259563, 1.403400, 4.029296, 0.009724, 0.02},
		{0.289257, 1.483987, 4.027409, 0.009659, 0.02},
		{0.320543, 1.564666, 4.034856, 0.009582, 0.02},
		{0.353459, 1.645410, 4.036300, 0.009494, 0.02},
		{0.387963, 1.726225, 4.043050, 0.009391, 0.02},
		{0.424106, 1.807122, 4.044802, 0.009274, 0.02},
		{0.461882, 1.888157, 4.050442, 0.009139, 0.02},
		{0.501252, 1.969288, 4.058121, 0.008986, 0.02},
		{0.542267, 2.050536, 4.061931, 0.008812, 0.02},
		{0.584919, 2.131950, 4.069542, 0.008614, 0.02},
		{0.629189, 2.213518, 4.078391, 0.008392, 0.02},
		{0.675095, 2.295251, 4.086589, 0.008140, 0.02},
		{0.722640, 2.377185, 4.096591, 0.007856, 0.02},
		{0.771815, 2.459336, 4.108483, 0.007537, 0.02},
		{0.822640, 2.541733, 4.120646, 0.007178, 0.02},
		{0.875138, 2.624435, 4.134411, 0.006775, 0.02},
		{0.929293, 2.707475, 4.151581, 0.006321, 0.02},
		{0.985112, 2.790872, 4.169713, 0.005811, 0.02},
		{1.042603, 2.874663, 4.189733, 0.005238, 0.02},
		{1.101773, 2.958881, 4.211423, 0.004596, 0.02},
		{1.162645, 3.043553, 4.233498, 0.003876, 0.02},
		{1.225225, 3.128682, 4.256043, 0.003072, 0.02},
		{1.289501, 3.214201, 4.276519, 0.002176, 0.02},
		{1.355506, 3.299993, 4.289229, 0.001185, 0.02},
		{1.423222, 3.385824, 4.291564, 0.000097, 0.02},
		{1.492616, 3.470154, 4.217080, -0.001080, 0.02},
		{1.562624, 3.499957, 1.489972, -0.002312, 0.02},
		{1.632614, 3.499955, -0.000137, -0.003558, 0.02},
		{1.702619, 3.499952, -0.000149, -0.004777, 0.02},
		{1.772613, 3.499952, 0.000040, -0.005919, 0.02},
		{1.842617, 3.499954, 0.000101, -0.006930, 0.02},
		{1.912605, 3.499954, -0.000001, -0.007758, 0.02},
		{1.982615, 3.499963, 0.000415, -0.008364, 0.02},
		{2.052607, 3.499954, -0.000422, -0.008728, 0.02},
		{2.122609, 3.499954, -0.000020, -0.008849, 0.02},
		{2.192368, 3.488222, -0.586610, -0.008753, 0.02},
		{2.261708, 3.466482, -1.086875, -0.008476, 0.02},
		{2.330704, 3.449777, -0.835250, -0.008063, 0.02},
		{2.398599, 3.395383, -2.720156, -0.007562, 0.02},
		{2.464863, 3.312669, -4.135072, -0.007017, 0.02},
		{2.529501, 3.231720, -4.047243, -0.006456, 0.02},
		{2.592542, 3.152006, -3.985591, -0.005899, 0.02},
		{2.653994, 3.073110, -3.945468, -0.005362, 0.02},
		{2.713899, 2.994704, -3.919589, -0.004850, 0.02},
		{2.772216, 2.916579, -3.907225, -0.004369, 0.02},
		{2.828994, 2.838585, -3.899255, -0.003921, 0.02},
		{2.884206, 2.760613, -3.898689, -0.003507, 0.02},
		{2.937863, 2.682608, -3.899891, -0.003126, 0.02},
		{2.989957, 2.604530, -3.903676, -0.002777, 0.02},
		{3.040479, 2.526365, -3.908625, -0.002458, 0.02},
		{3.089439, 2.448100, -3.913389, -0.002168, 0.02},
		{3.136844, 2.369714, -3.918430, -0.001905, 0.02},
		{3.182673, 2.291214, -3.924617, -0.001666, 0.02},
		{3.226918, 2.212624, -3.930140, -0.001451, 0.02},
		{3.269587, 2.133945, -3.934843, -0.001258, 0.02},
		{3.310699, 2.055156, -3.938607, -0.001084, 0.02},
		{3.350216, 1.976274, -3.944909, -0.000929, 0.02},
		{3.388180, 1.897297, -3.947005, -0.000791, 0.02},
		{3.424546, 1.818230, -3.953181, -0.000669, 0.02},
		{3.459328, 1.739105, -3.956317, -0.000561, 0.02},
		{3.492520, 1.659919, -3.960105, -0.000467, 0.02},
		{3.524124, 1.580674, -3.963308, -0.000385, 0.02},
		{3.554151, 1.501359, -3.965871, -0.000314, 0.02},
		{3.582613, 1.421938, -3.967720, -0.000253, 0.02},
		{3.609458, 1.342465, -3.974368, -0.000201, 0.02},
		{3.634707, 1.262987, -3.975530, -0.000157, 0.02},
		{3.658390, 1.183423, -3.975742, -0.000121, 0.02},
		{3.680464, 1.103798, -3.981710, -0.000091, 0.02},
		{3.700937, 1.024168, -3.983370, -0.000067, 0.02},
		{3.719824, 0.944495, -3.984336, -0.000048, 0.02},
		{3.737139, 0.864712, -3.984382, -0.000034, 0.02},
		{3.752824, 0.784915, -3.993130, -0.000023, 0.02},
		{3.766924, 0.705137, -3.989766, -0.000014, 0.02},
		{3.779434, 0.625278, -3.991653, -0.000009, 0.02},
		{3.790350, 0.545347, -3.993257, -0.000005, 0.02},
		{3.799670, 0.465343, -3.994510, -0.000002, 0.02},
		{3.807367, 0.385402, -4.002480, -0.000001, 0.02},
		{3.813491, 0.305392, -3.989984, -0.000000, 0.02},
		{3.817991, 0.225383, -4.007619, -0.000000, 0.02},
		{3.820916, 0.145172, -3.981160, -0.000000, 0.02},
		{3.822216, 0.064687, -4.004897, -0.000000, 0.02}};
		
	public static double [][]RightPoints = new double[][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.000825, 0.041251, 2.062491, 0.010000, 0.02},
		{0.003225, 0.120000, 3.937383, 0.010000, 0.02},
		{0.007225, 0.199994, 3.999591, 0.010000, 0.02},
		{0.012825, 0.279976, 3.999113, 0.009999, 0.02},
		{0.020023, 0.359942, 3.998465, 0.009998, 0.02},
		{0.028845, 0.439990, 3.992461, 0.009996, 0.02},
		{0.039238, 0.519993, 4.002639, 0.009993, 0.02},
		{0.051226, 0.599843, 3.995404, 0.009989, 0.02},
		{0.064806, 0.679650, 3.994242, 0.009982, 0.02},
		{0.079998, 0.759462, 3.989824, 0.009973, 0.02},
		{0.096798, 0.839310, 3.989254, 0.009960, 0.02},
		{0.115172, 0.919094, 3.990880, 0.009944, 0.02},
		{0.135137, 0.998785, 3.986650, 0.009923, 0.02},
		{0.156706, 1.078448, 3.983077, 0.009897, 0.02},
		{0.179866, 1.158080, 3.981968, 0.009865, 0.02},
		{0.204623, 1.237666, 3.978649, 0.009827, 0.02},
		{0.230983, 1.317222, 3.975555, 0.009780, 0.02},
		{0.258895, 1.396682, 3.975973, 0.009724, 0.02},
		{0.288431, 1.476074, 3.967664, 0.009659, 0.02},
		{0.319532, 1.555416, 3.968015, 0.009582, 0.02},
		{0.352234, 1.634672, 3.961848, 0.009494, 0.02},
		{0.386490, 1.713830, 3.960188, 0.009391, 0.02},
		{0.422349, 1.792888, 3.952824, 0.009274, 0.02},
		{0.459798, 1.871880, 3.948328, 0.009139, 0.02},
		{0.498798, 1.950744, 3.944724, 0.008986, 0.02},
		{0.539392, 2.029474, 3.936101, 0.008812, 0.02},
		{0.581566, 2.108091, 3.929707, 0.008614, 0.02},
		{0.625297, 2.186547, 3.922805, 0.008392, 0.02},
		{0.670594, 2.264815, 3.913328, 0.008140, 0.02},
		{0.717453, 2.342884, 3.903335, 0.007856, 0.02},
		{0.765856, 2.420718, 3.892583, 0.007537, 0.02},
		{0.815812, 2.498287, 3.879201, 0.007178, 0.02},
		{0.867333, 2.575582, 3.864116, 0.006775, 0.02},
		{0.920389, 2.652564, 3.848723, 0.006321, 0.02},
		{0.974975, 2.729178, 3.830551, 0.005811, 0.02},
		{1.031080, 2.805386, 3.810541, 0.005238, 0.02},
		{1.088696, 2.881155, 3.788961, 0.004596, 0.02},
		{1.147827, 2.956474, 3.765861, 0.003876, 0.02},
		{1.208460, 3.031355, 3.743675, 0.003072, 0.02},
		{1.270568, 3.105833, 3.724374, 0.002176, 0.02},
		{1.334174, 3.180037, 3.709920, 0.001185, 0.02},
		{1.399258, 3.254213, 3.708799, 0.000097, 0.02},
		{1.465803, 3.327679, 3.673793, -0.001080, 0.02},
		{1.532830, 3.350911, 1.161473, -0.002312, 0.02},
		{1.599805, 3.349212, -0.084955, -0.003558, 0.02},
		{1.666860, 3.352471, 0.162920, -0.004777, 0.02},
		{1.734091, 3.361769, 0.464919, -0.005919, 0.02},
		{1.801648, 3.377636, 0.793293, -0.006930, 0.02},
		{1.869631, 3.399714, 1.104114, -0.007758, 0.02},
		{1.938175, 3.426613, 1.344698, -0.008364, 0.02},
		{2.007288, 3.456006, 1.469840, -0.008728, 0.02},
		{2.076995, 3.485213, 1.460279, -0.008849, 0.02},
		{2.146989, 3.499947, 0.736769, -0.008753, 0.02},
		{2.216998, 3.499967, 0.000990, -0.008476, 0.02},
		{2.286993, 3.499725, -0.012129, -0.008063, 0.02},
		{2.356100, 3.455955, -2.188861, -0.007562, 0.02},
		{2.423683, 3.378670, -3.863667, -0.007017, 0.02},
		{2.489679, 3.299602, -3.953178, -0.006456, 0.02},
		{2.554067, 3.219307, -4.014661, -0.005899, 0.02},
		{2.616821, 3.138214, -4.055333, -0.005362, 0.02},
		{2.677964, 3.056617, -4.079094, -0.004850, 0.02},
		{2.737444, 2.974746, -4.094568, -0.004369, 0.02},
		{2.795305, 2.892750, -4.099379, -0.003921, 0.02},
		{2.851519, 2.810713, -4.101875, -0.003507, 0.02},
		{2.906098, 2.728711, -4.099782, -0.003126, 0.02},
		{2.959037, 2.646776, -4.096463, -0.002777, 0.02},
		{3.010331, 2.564942, -4.092107, -0.002458, 0.02},
		{3.059993, 2.483217, -4.086414, -0.002168, 0.02},
		{3.108036, 2.401587, -4.080581, -0.001905, 0.02},
		{3.154441, 2.320060, -4.075960, -0.001666, 0.02},
		{3.199207, 2.238654, -4.070934, -0.001451, 0.02},
		{3.242344, 2.157363, -4.065502, -0.001258, 0.02},
		{3.283877, 2.076153, -4.059613, -0.001084, 0.02},
		{3.323769, 1.995033, -4.056877, -0.000929, 0.02},
		{3.362066, 1.913988, -4.050343, -0.000791, 0.02},
		{3.398729, 1.833014, -4.048501, -0.000669, 0.02},
		{3.433770, 1.752136, -4.044014, -0.000561, 0.02},
		{3.467191, 1.671339, -4.040627, -0.000467, 0.02},
		{3.498994, 1.590620, -4.037036, -0.000385, 0.02},
		{3.529193, 1.509959, -4.033154, -0.000314, 0.02},
		{3.557803, 1.429314, -4.028870, -0.000253, 0.02},
		{3.584773, 1.348734, -4.029752, -0.000201, 0.02},
		{3.610128, 1.268260, -4.025344, -0.000157, 0.02},
		{3.633898, 1.187806, -4.020240, -0.000121, 0.02},
		{3.656044, 1.107391, -4.021206, -0.000091, 0.02},
		{3.676575, 1.027068, -4.018061, -0.000067, 0.02},
		{3.695508, 0.946792, -4.014467, -0.000048, 0.02},
		{3.712858, 0.866492, -4.010199, -0.000034, 0.02},
		{3.728571, 0.786259, -4.014949, -0.000023, 0.02},
		{3.742690, 0.706120, -4.007801, -0.000014, 0.02},
		{3.755214, 0.625970, -4.006228, -0.000009, 0.02},
		{3.766139, 0.545809, -4.004689, -0.000005, 0.02},
		{3.775465, 0.465633, -4.003135, -0.000002, 0.02},
		{3.783166, 0.385568, -4.008675, -0.000001, 0.02},
		{3.789291, 0.305476, -3.994101, -0.000000, 0.02},
		{3.793791, 0.225418, -4.010093, -0.000000, 0.02},
		{3.796717, 0.145181, -3.982384, -0.000000, 0.02},
		{3.798017, 0.064688, -4.005322, -0.000000, 0.02}};
}