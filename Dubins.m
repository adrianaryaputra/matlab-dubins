classdef Dubins < handle
%Dubins Finder from array of waypoint coordinate.
%by : Adrian A. Firmansyah [http://github.com/kron3]
%     
%constructor: Dubins(array of waypoint[x y],dubinsRadius,softingPoint)
%result : array of dubins 
%> greater softingPoint make dubins circle softer but require more computing
%> result can be accessed in Dubins.result [array of<struct: x y>]
%     
%example :
%waypoint = [1 1 ; 4 3 ; 8 7]; radius = 1;
%dubins = Dubins( waypoint , radius , 10);
%result = dubins.result;
    
    properties
        
        nodes           % group of waypoint[x y]
        heading         % heading (degree)
        circles         % group of circle<object: x y r rotation >
        circlesCalc     % group of calculationResult<struct>
        result          % group of resultCoordinate<struct: x y>
        
    end
    
    
    methods
        
        function this = Dubins(nodes, radius, softenPoint)
        %Initialize Dubins
        %params: Dubins(nodes[],radius,softenPoint)
            this.nodes = nodes;
            this.heading = this.findHeadings(nodes);
            this.circles = this.findCircles(radius);
            this.circlesCalc = this.circlesCalculate(this.circles);
            this.result = this.waypointCalculate(softenPoint);
        end
        
        function result = Circle(this,r,x,y,rotation)
        %Circle struct constructor for Dubins
        %params: Circle(r,x,y,rotation)
        %return: <struct: r x y rotation>
            result.r = r;
            result.x = x;
            result.y = y;
            result.rotation = rotation;
        end
        
        function result = findHeading(this, p1, p2)
        %Find Heading degree of a point
        %params: findHeading(point1,point2)
        %result: heading degree of point1
            if p2(1)-p1(1)>0
                if p2(2)-p1(2)<=0
                    result = -(180-(90+atand( abs(p2(2)-p1(2)) / abs(p2(1)-p1(1)) )));
                else
                    result = -(180-atand( abs(p2(1)-p1(1)) / abs(p2(2)-p1(2)) ));
                end
            else
                if p2(2)-p1(2)<=0
                    result = 180+(-90-atand( abs(p2(2)-p1(2)) / abs(p2(1)-p1(1)) ));
                else
                    result = 180-atand( abs(p2(1)-p1(1)) / abs(p2(2)-p1(2)) );
                end
            end
        end %fn findHeading()
        
        function result = findHeadings(this,w)
        %calculate heading of all node in waypoint
        %params: findHeadingAll(waypoint)
        %return: headingDegree[array]
            result = [];
            for i=1:size(w,1)-1
                result(i+1) = this.findHeading(w(i,:),w(i+1,:));
            end
        end %fn findHeadingAll()
        
        function result = findCircleP(this,r,n,h,hr)
        %find circle midpoint for dubins
        %params: findCircleP(radius,node,heading,headingReference)
        %result: coordinate[x y]
            if sind(h-hr) == 0
                rr = 0;
                x = n(1) + 0.5;
                y = n(2) + 0.5;
                hd = 0;
            elseif sind(h-hr) > 0
                rr = r;
                x = n(1) - ( r*cos(deg2rad(h)+(pi)) ) + 0.5;
                y = n(2) + ( r*sin(deg2rad(h)+(pi)) ) + 0.5;
                hd = 1;
            else
                rr = r;
                x = n(1) - ( r*cos(deg2rad(h)+(0)) ) + 0.5;
                y = n(2) + ( r*sin(deg2rad(h)+(0)) ) + 0.5;
                hd = 0;
            end
            result = this.Circle(rr,x,y,hd);
        end
        
        function result = findCircles(this,r)
        %find all circles (iterating through all nodes[])
        %params : findCircles(circleRadius)
        %return : circles[array of <struct: x y r>]
            result = [ this.findCircleP(0,this.nodes(1,:),this.heading(1),this.heading(1)) ];
            for i=2:size(this.nodes,1)-1
                result = [ result this.findCircleP(r,this.nodes(i,:),this.heading(i),this.heading(i+1))  ];
            end
            result = [ result this.findCircleP(0,this.nodes(end,:),this.heading(end),this.heading(end)) ];
        end
        
        function result = circleCalculate(this,cs,cf)
        % calculate tangent in & tangent out for trajectory between a
        % circle with another reference circle.
        % params : circleCalculate(this , thisCircle<struct: x y r> , referenceCircle<struct: x y r> )
        % return : <struct: distance , angleOfMidpoint , angleOfRadius , thisExitAngle , referenceEntryAngle>
            distance = sqrt((cs.x-cf.x)^2 + (cs.y-cf.y)^2);
            if cf.rotation == 1 %CCW
                if cs.rotation == 1 %CCW
                    psi = atan((cf.y-cs.y)/(cf.x-cs.x));
                    phi = asin((cf.r-cs.r)/distance);
                    phiSinggung = phi - pi/2 + psi;
                    phiNext = phiSinggung;
                    ten = [ cf.x+cf.r*cos(phiSinggung) cf.y+cf.r*sin(phiSinggung) ];
                    tex = [ cs.x+cs.r*cos(phiSinggung) cs.y+cs.r*sin(phiSinggung) ];
                else %CW
                    psi = atan((cf.y-cs.y)/(cf.x-cs.x));
                    phi = acos((cs.r+cf.r)/distance);
                    phiSinggung = phi + psi;
                    phiNext = phiSinggung - pi;
                    ten = [ cf.x+cf.r*cos(phiSinggung-pi) cf.y+cf.r*sin(phiSinggung-pi) ];
                    tex = [ cs.x+cs.r*cos(phiSinggung) cs.y+cs.r*sin(phiSinggung) ];
                end
            else %CW
                if cs.rotation == 1 %CCW
                    psi = atan((cf.y-cs.y)/(cf.x-cs.x));
                    phi = -acos((cs.r+cf.r)/distance);
                    phiSinggung = phi + psi;%-pi + phi - psi;
                    phiNext = phiSinggung+pi;
                    ten = [ cf.x+cf.r*cos(phiNext) cf.y+cf.r*sin(phiNext) ];
                    tex = [ cs.x+cs.r*cos(phiSinggung) cs.y+cs.r*sin(phiSinggung) ];
                else %CW
                    psi = atan((cf.y-cs.y)/(cf.x-cs.x));
                    phi = asin((cf.r-cs.r)/distance);
                    phiSinggung = phi + pi/2 + psi;
                    phiNext = phiSinggung;
                    ten = [ cf.x+cf.r*cos(phiSinggung) cf.y+cf.r*sin(phiSinggung) ];
                    tex = [ cs.x+cs.r*cos(phiSinggung) cs.y+cs.r*sin(phiSinggung) ];
                end
            end
            
            result = struct('d',distance,'psi',psi,'phi',phi,'radianExit',phiSinggung,'radianEntry',phiNext,'pointEntry',ten,'pointExit',tex);
        end
        
        function result = circlesCalculate(this,circles)
        %calculate all circle in given array of Circle<struct: x y r>
        %params : circlesCalculate(this,circles[array of <struct: x y r>])
        %return : [ array of <struct: distance , angleOfMidpoint ,
        %angleOfRadius , thisExitAngle , referenceEntryAngle> ]
            result = [];
            for i=1:length(circles)-1
                result = [result this.circleCalculate(circles(i),circles(i+1))];
            end
        end
        
        function result = waypointCalculate(this,divider)
            result = [struct('x',this.circles(1).x,'y',this.circles(1).y)];
            for i=2:length(this.circlesCalc)
                entry = this.circlesCalc(i-1).radianEntry;
                exit  = this.circlesCalc(i).radianExit;
                mid = this.circles(i);
                for j=0:divider
                    x = mid.x + mid.r*cos(entry+((exit-entry)/divider)*j);
                    y = mid.y + mid.r*sin(entry+((exit-entry)/divider)*j);
                    result = [result struct('x',x,'y',y)];
                end
            end
            
            result(end+1) = struct('x',this.circles(end).x,'y',this.circles(end).y)
            similar = logical(zeros(1,length(result)));
            % filter similar
            for m=1:length(result)
                for n=m+1:length(result)
                    if isequaln(result(m),result(n))
                        similar(n) = similar(n) | 1;
                        result(n) = struct('x',Inf,'y',Inf);
                    else
                        similar(n) = similar(n) | 0;
                    end
                end
            end
            
            result = result(~similar);
            
        end
        
    end
end

