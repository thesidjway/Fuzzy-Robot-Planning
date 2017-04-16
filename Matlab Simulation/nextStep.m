%% nextStep: Computes the next step in the robot's path
function [X, Y, theta] = nextStep(XOld, YOld, thetaOld, VR, VL, T, L)
	X = XOld + T * ((VR + VL) / 2) * cos(thetaOld * pi / 180);
	Y = YOld + T * ((VR + VL) / 2) * sin(thetaOld * pi / 180);
	theta = thetaOld + T * ((VR - VL) / L) * 180 / pi;
	if theta > 180
		theta = theta - 360;
	elseif theta < -180
		theta = theta +360;
	end
end
