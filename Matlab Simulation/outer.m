%% outer: The main function
function [] = outer(x, y, theta, goalX, goalY, T, L, height, width)
	d = sqrt((goalX - x)^2 + (goalY - y)^2);
	pathx = [x];
	pathy = [y];
	paththeta = [theta];
	vr = [0];
	vl = [0];
	while d > (0.1)
		[VR, VL] = FLC(x, y, goalX, goalY, theta);
		[x, y, theta] = nextStep(x, y, theta, VR, VL, T, L);
		pathx = [pathx ; x];
		pathy = [pathy ; y];
		vr = [vr; VR];
		vl = [vl; VL];
		paththeta = [paththeta; theta];
		d = sqrt((goalX - x)^2 + (goalY - y)^2);
		il = size(pathx, 1);
		clf;
		plot(pathx, pathy, 'b');
		hold on;
		pause(T);
	end
end