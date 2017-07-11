clear all;
clc;
format longG;

list = [];

a = body;
a.name = 'Base';
a.V = 668293.50498;
a.roh = 2.8e-6;
a.m = a.V * a.roh;
a.CoM = [-3.61483029333E-07 12.7334577336 87.9729092856]';
a.I = [ 7448530703.95   -59.306059444   -40.7095603448;
        -59.306059444   7605678914.8    606902711.949;
        -40.7095603448  606902711.949   6841499590.22] .* a.roh;
list = [list, a];


comp = compound_bodys(list);
comp.CoM = comp.CoM * 1e-3; % from mm to m
comp.I = comp.I * 1e-6; % from kg mm² to kg m²