clear all;
clc;
format longG;

list = [];

a = body;
a.name = 'Gabelkopf';
a.V = 247.475080019;
a.roh = 2.8e-6;
a.m = a.V * a.roh;
a.CoM = [-2.57691018276E-13	2.8768259623E-12	12.0730514315]';
a.I = [ 5078.78840569	-1.0811664064E-14	8.48992839848E-13;
        -1.0811664064E-14	4715.34444964	-4.26993105629E-13;
        8.48992839848E-13	-4.26993105629E-13	1912.89105707] .* a.roh;
list = [list, a];

b = a.copy();
b.rotateAroundAxis([0 0 0]', [0 1 0]', pi);
list = [list, b];


c = body;
c.name = 'Mutter_oben';
c.V = 18.8903810816;
c.roh = 7.55e-6; % m =1.42 g
c.m = c.V * c.roh;
c.CoM = [-0.0662748639	3.7968844079	18]';
c.I = [ 28.1078461894	-0.3505376333	0;
        -0.3505376333	48.1840150962	0;
        0           0       28.1017275323] .* c.roh;
list = [list, c];

d = c.copy();
d.rotateAroundAxis([0 0 0]', [0 1 0]', pi);
list = [list, d];

e = body;
e.name = 'Schraube_oben';
e.V = 44.6748506925;
e.roh = 5.8e-6;
e.m = e.V * e.roh;
e.CoM = [-3.2684965845E-13	-1.1921036258	18]';
e.I = [ 433.58500178	8.73114913702E-11	2.32830643654E-10;
        8.73114913702E-11	52.1467146487	1.16415321827E-10;
        2.32830643654E-10	1.16415321827E-10	433.58500178] .* e.roh;
list = [list, e];

f = e.copy();
f.rotateAroundAxis([0 0 0]', [0 1 0]', pi);
list = [list, f];


g = body;
g.name = 'Distanzhülse_oben';
g.V = 15.707963268;
g.roh = 2.8e-6;
g.m = g.V * g.roh;
g.CoM = [1.98951966013E-13	3.60955709766E-12	5]';
g.I = [ 33.7066711791       0               -1.45519152284E-11  ;
        0                   33.706671179	0                   ;
        -1.45519152284E-11	0               25.5254403105       ] .* g.roh;
list = [list, g];

h = g.copy();
h.rotateAroundAxis([0 0 0]', [0 1 0]', pi);
list = [list, h];

i = body;
i.name = 'Gewindestange';
i.V = 68.7550642208;
i.roh = 5.8e-6;
i.m = i.V * i.roh;
i.CoM = [-3.8546943415E-13	2.90611978926E-12	-3.12638803734E-12]';
i.I = [ 2761.87974191	1.81898940355E-12	3.63797880709E-12;
        1.81898940355E-12	2761.87974191	-5.82076609135E-11;
        3.63797880709E-12	-5.82076609135E-11	34.2513366241] .* i.roh;
list = [list, i];

comp = compound_bodys(list);
% link 2
% comp.CoM = comp.CoM + [0 0 0]';
% r = roty(90, 'deg');
% link 4
comp.CoM = comp.CoM + [0 0 18]';
r = rotz(90, 'deg') * roty(-90, 'deg');

comp.CoM = r * comp.CoM;
comp.I = r * comp.I * r';
comp.CoM = comp.CoM * 1e-3; % from mm to m
comp.I = comp.I * 1e-6; % from kg mm² to kg m²
