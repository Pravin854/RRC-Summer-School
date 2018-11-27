% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 662.495252165942361 ; 664.677385420665473 ];

%-- Principal point:
cc = [ 306.512714725873877 ; 241.751010274133932 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.279075054807933 ; 0.320250080616207 ; 0.000504407168731 ; 0.000278160138618 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.434014187314598 ; 1.542623164226049 ];

%-- Principal point uncertainty:
cc_error = [ 2.834859507785982 ; 2.608302331444571 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.011436817050496 ; 0.047293880156067 ; 0.000643544703798 ; 0.000669414163677 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.653090e+00 ; 1.646178e+00 ; -6.680988e-01 ];
Tc_1  = [ -6.072620e+02 ; -2.746388e+02 ; 2.860014e+03 ];
omc_error_1 = [ 3.288790e-03 ; 4.270825e-03 ; 5.442548e-03 ];
Tc_error_1  = [ 1.228368e+01 ; 1.134346e+01 ; 6.406053e+00 ];

%-- Image #2:
omc_2 = [ 1.845814e+00 ; 1.894593e+00 ; -3.972580e-01 ];
Tc_2  = [ -5.303209e+02 ; -5.268186e+02 ; 2.543707e+03 ];
omc_error_2 = [ 3.489021e-03 ; 4.285818e-03 ; 6.637268e-03 ];
Tc_error_2  = [ 1.098650e+01 ; 1.008516e+01 ; 6.286114e+00 ];

%-- Image #3:
omc_3 = [ 1.739433e+00 ; 2.071099e+00 ; -5.057408e-01 ];
Tc_3  = [ -4.314212e+02 ; -5.774973e+02 ; 2.603770e+03 ];
omc_error_3 = [ 3.192407e-03 ; 4.538089e-03 ; 6.861824e-03 ];
Tc_error_3  = [ 1.123454e+01 ; 1.032067e+01 ; 6.033503e+00 ];

%-- Image #4:
omc_4 = [ 1.826231e+00 ; 2.110301e+00 ; -1.097720e+00 ];
Tc_4  = [ -2.290345e+02 ; -5.119042e+02 ; 2.614246e+03 ];
omc_error_4 = [ 2.858203e-03 ; 4.661942e-03 ; 6.368423e-03 ];
Tc_error_4  = [ 1.129301e+01 ; 1.029475e+01 ; 4.867799e+00 ];

%-- Image #5:
omc_5 = [ 2.175841e+00 ; 6.094789e-01 ; 6.197688e-01 ];
Tc_5  = [ -5.642536e+02 ; -1.184406e+02 ; 1.614412e+03 ];
omc_error_5 = [ 4.420831e-03 ; 2.690525e-03 ; 5.195259e-03 ];
Tc_error_5  = [ 7.130976e+00 ; 6.507280e+00 ; 5.816766e+00 ];

%-- Image #6:
omc_6 = [ -1.699614e+00 ; -1.931158e+00 ; -7.983276e-01 ];
Tc_6  = [ -5.043948e+02 ; -2.628242e+02 ; 1.494148e+03 ];
omc_error_6 = [ 2.710268e-03 ; 4.357810e-03 ; 5.889316e-03 ];
Tc_error_6  = [ 6.456677e+00 ; 6.068836e+00 ; 5.095278e+00 ];

%-- Image #7:
omc_7 = [ 1.991862e+00 ; 1.931445e+00 ; 1.314183e+00 ];
Tc_7  = [ -2.849586e+02 ; -2.570408e+02 ; 1.484646e+03 ];
omc_error_7 = [ 5.133082e-03 ; 2.638399e-03 ; 6.158850e-03 ];
Tc_error_7  = [ 6.516928e+00 ; 5.958628e+00 ; 5.378108e+00 ];

%-- Image #8:
omc_8 = [ 1.958143e+00 ; 1.824826e+00 ; 1.327689e+00 ];
Tc_8  = [ -5.776987e+02 ; -3.433137e+02 ; 1.559802e+03 ];
omc_error_8 = [ 4.862701e-03 ; 2.676365e-03 ; 5.875270e-03 ];
Tc_error_8  = [ 7.162889e+00 ; 6.488357e+00 ; 6.052846e+00 ];

%-- Image #9:
omc_9 = [ -1.368193e+00 ; -1.987335e+00 ; 3.162264e-01 ];
Tc_9  = [ -1.914466e+01 ; -7.474219e+02 ; 2.451498e+03 ];
omc_error_9 = [ 3.342775e-03 ; 4.284681e-03 ; 5.571599e-03 ];
Tc_error_9  = [ 1.064704e+01 ; 9.687883e+00 ; 6.173134e+00 ];

%-- Image #10:
omc_10 = [ -1.518485e+00 ; -2.092998e+00 ; 1.820972e-01 ];
Tc_10  = [ -1.145208e+02 ; -9.970945e+02 ; 2.891663e+03 ];
omc_error_10 = [ 4.083483e-03 ; 4.883862e-03 ; 7.450017e-03 ];
Tc_error_10  = [ 1.278795e+01 ; 1.150847e+01 ; 8.200149e+00 ];

%-- Image #11:
omc_11 = [ -1.792271e+00 ; -2.066014e+00 ; -4.884751e-01 ];
Tc_11  = [ -5.164092e+02 ; -7.794147e+02 ; 2.364279e+03 ];
omc_error_11 = [ 3.661817e-03 ; 4.600739e-03 ; 7.918950e-03 ];
Tc_error_11  = [ 1.046309e+01 ; 9.842440e+00 ; 8.084088e+00 ];

%-- Image #12:
omc_12 = [ -1.837717e+00 ; -2.088344e+00 ; -5.240869e-01 ];
Tc_12  = [ -4.559239e+02 ; -5.866572e+02 ; 2.029762e+03 ];
omc_error_12 = [ 3.142949e-03 ; 4.448891e-03 ; 7.341438e-03 ];
Tc_error_12  = [ 8.907954e+00 ; 8.313651e+00 ; 6.766155e+00 ];

%-- Image #13:
omc_13 = [ -1.917005e+00 ; -2.117199e+00 ; -6.024017e-01 ];
Tc_13  = [ -4.522069e+02 ; -4.750265e+02 ; 1.828713e+03 ];
omc_error_13 = [ 2.945086e-03 ; 4.413315e-03 ; 7.217846e-03 ];
Tc_error_13  = [ 8.000771e+00 ; 7.444511e+00 ; 6.149098e+00 ];

%-- Image #14:
omc_14 = [ -1.952778e+00 ; -2.125633e+00 ; -5.928971e-01 ];
Tc_14  = [ -4.208507e+02 ; -4.540356e+02 ; 1.648664e+03 ];
omc_error_14 = [ 2.764124e-03 ; 4.306279e-03 ; 7.048432e-03 ];
Tc_error_14  = [ 7.227125e+00 ; 6.710721e+00 ; 5.522120e+00 ];

%-- Image #15:
omc_15 = [ -2.110890e+00 ; -2.256300e+00 ; -4.850643e-01 ];
Tc_15  = [ -6.732627e+02 ; -4.482697e+02 ; 1.617355e+03 ];
omc_error_15 = [ 3.185452e-03 ; 3.969863e-03 ; 7.642041e-03 ];
Tc_error_15  = [ 7.229477e+00 ; 6.740576e+00 ; 5.906550e+00 ];

%-- Image #16:
omc_16 = [ 1.885473e+00 ; 2.332090e+00 ; -1.540358e-01 ];
Tc_16  = [ -6.569770e+01 ; -5.654837e+02 ; 2.337128e+03 ];
omc_error_16 = [ 4.304641e-03 ; 4.538682e-03 ; 9.164109e-03 ];
Tc_error_16  = [ 1.007996e+01 ; 9.178634e+00 ; 7.045460e+00 ];

%-- Image #17:
omc_17 = [ -1.613391e+00 ; -1.956913e+00 ; -3.563788e-01 ];
Tc_17  = [ -4.596269e+02 ; -4.600820e+02 ; 1.644781e+03 ];
omc_error_17 = [ 2.723344e-03 ; 4.144649e-03 ; 5.843500e-03 ];
Tc_error_17  = [ 7.131866e+00 ; 6.653576e+00 ; 4.921402e+00 ];

%-- Image #18:
omc_18 = [ -1.344915e+00 ; -1.706741e+00 ; -3.090760e-01 ];
Tc_18  = [ -6.214216e+02 ; -5.210788e+02 ; 1.496077e+03 ];
omc_error_18 = [ 3.083633e-03 ; 4.041639e-03 ; 4.624876e-03 ];
Tc_error_18  = [ 6.541776e+00 ; 6.123426e+00 ; 4.736484e+00 ];

%-- Image #19:
omc_19 = [ -1.923525e+00 ; -1.840209e+00 ; -1.442396e+00 ];
Tc_19  = [ -3.635455e+02 ; -2.636507e+02 ; 1.131245e+03 ];
omc_error_19 = [ 2.681460e-03 ; 4.684718e-03 ; 5.921668e-03 ];
Tc_error_19  = [ 5.079886e+00 ; 4.678617e+00 ; 4.489803e+00 ];

%-- Image #20:
omc_20 = [ 1.892439e+00 ; 1.593457e+00 ; 1.472738e+00 ];
Tc_20  = [ -4.891758e+02 ; -2.921673e+02 ; 1.339675e+03 ];
omc_error_20 = [ 4.928086e-03 ; 2.736448e-03 ; 5.315962e-03 ];
Tc_error_20  = [ 6.220683e+00 ; 5.562542e+00 ; 5.413725e+00 ];

