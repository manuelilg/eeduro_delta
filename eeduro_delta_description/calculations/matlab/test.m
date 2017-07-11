clear all;
clc;

list = [];

a = body;
a.name = 'Body1';
a.V = 100;
a.roh = 1;
a.m = a.V * a.roh;
a.CoM = [10 0 0]';
a.I = [100 0 0; 0 100 0; 0 0 100];
list = [list, a];

% b = body;
% b.name = 'Body1';
% b.V = 100;
% b.roh = 1;
% b.m = b.V * b.roh;
% b.CoM = [-10 0 0]';
% b.I = [100 0 0; 0 100 0; 0 0 100];
% list = [list, b];

% b= a.copy();
% b.CoM = [-10 0 0]';
% list = [list, b];

b= a.copy();
b.rotateAroundAxis([0 0 0]', [0 0 1]', 4*pi/6);
list = [list, b];

c= b.copy();
c.rotateAroundAxis([0 0 0]', [0 0 1]', 4*pi/6);
list = [list, c];


d = body;
d.CoM = [0.5 1 1.5]';
d.I = [6.5 0 0; 0 5 0; 0 0 2.5];
d.rotateAroundAxis([0 0 0]', [0 1 1]', 50/180*pi);

comp = compound_bodys(list)