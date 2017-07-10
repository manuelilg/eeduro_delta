function ass = compound_bodys(list)
ass = body;
ass.name = 'Compund Body';
ass.V = 0;
ass.roh = NaN;
ass.m = 0;
ass.CoM = [0 0 0]';
ass.I = [0 0 0; 0 0 0; 0 0 0];

for b = list
    ass.V = ass.V + b.V; 
    ass.CoM = (ass.CoM * ass.m + b.CoM * b.m)/(ass.m + b.m);
    ass.m = ass.m + b.m;
end


for b = list
    v = ass.CoM - b.CoM;
    ass.I = ass.I + b.I + b.m .* (dot(v,v) * eye(3,3) - v * v');
end


end