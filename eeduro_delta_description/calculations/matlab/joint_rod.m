clear all;
clc;
format longG;

list = [];

a = body;
a.name = 'Gelenkstange_Hülse';
a.V = 282.743338823;
a.roh = 1.55e-6;
a.m = a.V * a.roh;
a.CoM = [50	-3.55981910616E-11	-6.18172180111E-11]';
a.I = [ 459.457925583	3.41096892953E-08	4.7089997679E-08;
        3.41096892953E-08	122374.851334	-1.39698386192E-09;
        4.7089997679E-08	-1.39698386192E-09	122374.851334] .* a.roh;
list = [list, a];


b = body;
b.name = 'Gewindestange';
b.V = 266.675401397;
b.roh = 5.8e-6;
b.m = b.V * b.roh;
b.CoM = [50             6.8212102633E-13	9.23705556488E-14]';
b.I = [ 133.211505212	0                   -1.16415321827E-10;
        0               160196.555343       3.49245965481E-10;
        -1.16415321827E-10	3.49245965481E-10	160196.555343] .* b.roh;
list = [list, b];


c = body;
c.name = 'Mutter';
c.V = 18.8903810816;
c.roh = 7.55e-6; % m =1.42 g
c.m = c.V * c.roh;
c.CoM = [13.30253722	1.07363007373E-11	4.07283096138E-11]';
c.I = [ 48.190133753        -1.45519152284E-11	0;
        -1.45519152284E-11	28.101727532        1.45519152284E-11;
        0                   1.45519152284E-11	28.1017275323] .* c.roh;
list = [list, c];

d = c.copy();
d.rotateAroundAxis([50 0 0]', [0 0 1]', pi);
list = [list, d];


e = body;
e.name = 'Gelenkkopf';
e.V = 228.610841672;
e.roh = 1.37e-6; 
e.m = e.V * e.roh;
e.CoM = [4.98113701365E-16	9.02214512823E-14	-3.6597701864]';
e.I = [ 5544.09202336	-4.43189714663E-07	4.6875788026E-14;
        -4.43189714663E-07	6305.86423132	-4.03828369267E-11;
        4.6875788026E-14	-4.03828369267E-11	1179.58323727] .* e.roh;
e.rotateAroundAxis([0 0 0]', [0 1 0]', -pi/2);
list = [list, e];

f = e.copy();
f.rotateAroundAxis([50 0 0]', [0 0 1]', pi);
list = [list, f];


g = body;
g.name = 'Kalotte';
g.V = 55.627034336;
g.roh = 1.24e-6; 
g.m = g.V * g.roh;
g.CoM = [1.90153786353E-12	-3.00758499025E-13	1.16915853856E-15]';
g.I = [ 150.139374742	3.53918959537E-15	3.64257701109E-14;
        3.53918959537E-15	187.684387009	-1.1722686576E-13;
        3.64257701109E-14	-1.1722686576E-13	150.139329996] .* g.roh;
list = [list, g];

h = g.copy();
h.rotateAroundAxis([50 0 0]', [0 0 1]', pi);
list = [list, h];


comp = compound_bodys(list);
comp.CoM = comp.CoM + [0 0 0]';
r = rotz(90, 'deg') * roty(-90, 'deg');
comp.CoM = r * comp.CoM;
comp.I = r * comp.I * r';
comp.CoM = comp.CoM * 1e-3; % from mm to m
comp.I = comp.I * 1e-6; % from kg mm² to kg m²