clear all;
clc;
format longG;

list = [];

a = body;
a.name = 'Oberarm';
a.V = 3932.10358546;
a.roh = 2.8e-6;
a.m = a.V * a.roh;
a.CoM = [22.4067809439 -0.013768279 2.9178876322]';
a.I = [ 60548.5436791	-1213.06393042	8486.74273093;
        -1213.06393042	1047556.61565	4.4453725724;
        8486.74273093	4.4453725724	1085008.40434] .* a.roh;
list = [list, a];

b = body;
b.name = 'Kugellager';
b.V = 88.1216739332;
b.roh = 5.1e-6; %mass = 0.45g
b.m = b.V * b.roh;
b.CoM =  [50	-4.86721773996E-13	3.8588235294]';
b.I = [ 332.253967021	-2.91038304567E-11	-5.82076609135E-11;
        -2.91038304567E-11	332.253967021	-4.36557456851E-11;
        -5.82076609135E-11	-4.36557456851E-11	523.708404092] .* b.roh;
list = [list, b];

c = b.copy();
c.rotateAroundAxis([0 0 2.2]', [1 0 0]', pi);
list = [list, c];

d = body;
d.name = 'Gewindestift';
d.V = 22.4899889828;
d.roh = 7.75e-6;
d.m = d.V * d.roh;
d.CoM = [-3.8516242454	-4.40536496171E-13	3]';
d.I = [ 18.3441011634	9.09494701773E-13	3.63797880709E-12;
        9.09494701773E-13	51.4983489367	1.09139364213E-11;
        3.63797880709E-12	1.09139364213E-11	51.4983489368] .* d.roh;
list = [list, d];

e = d.copy();
e.rotateAroundAxis([0 0 3]', [0 0 1]', -pi/2);
list = [list, e];

comp = compound_bodys(list);
comp.CoM = comp.CoM + [0 0 -2.2]';
r = roty(-90, 'deg');
comp.CoM = r * comp.CoM;
comp.I = r * comp.I * r';
comp.CoM = comp.CoM * 1e-3; % from mm to m
comp.I = comp.I * 1e-6; % from kg mm² to kg m²
