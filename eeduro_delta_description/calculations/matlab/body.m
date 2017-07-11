classdef body<handle
    properties
        name        % string
        V           % Volume mm³
        roh         % density kg/mm³
        m           % Mass kg
        CoM         % Center of Mass [x; y; z] mm
        I           % IneriaMatrix kg mm²
    end
    
    properties (GetAccess=private)
        numberOfCopies = 0;
    end
    
    methods
        function cp = copy(obj)
           cp = body;
           cp.name = strcat(obj.name, '_copy', num2str(obj.numberOfCopies));
           obj.numberOfCopies = obj.numberOfCopies + 1;
           cp.V = obj.V;
           cp.roh = obj.roh;
           cp.m = obj.m;
           cp.CoM = obj.CoM;
           cp.I = obj.I;           
        end
                
        function rotateAroundAxis(obj, origin, axis, th)
            % angvec2r: function from robotics toolbox (Peter Corke)
            r = angvec2r(th, axis);
            obj.CoM = r*(obj.CoM - origin) + origin;
            obj.I = r*obj.I*r';
        end
    end
end