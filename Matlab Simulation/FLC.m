%% FLC: Fuzzy Logic Controller for part 1
function [VR, VL] = FLC(X, Y, XT, YT, theta)
	thetaT = atan2((YT - Y) , (XT - X)) * 180 / pi
	theta
	phi = thetaT - theta;
	d = sqrt((XT - X)^2 + (YT - Y)^2)
	a = readfis('part1.fis');
	if phi > 180
		phi = phi - 360;
	elseif phi < -180
		phi = phi + 360;
	end	
	phi
	x = evalfis([d, phi], a)
	VL = x(1);
	VR = x(2);
end