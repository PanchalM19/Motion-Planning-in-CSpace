% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    %Translate frame origins
    origin1_at0 = robot.pivot1;
    origin2_at0 = origin1_at0 + robot.pivot2;
    link1_at0 = robot.link1 + origin1_at0;
    link2_at0 = robot.link2 + origin1_at0;
    
    %Polygon corners
    polyin = polyshape(link1_at0(1,:), link1_at0(2,:));
    angle = rad2deg(q(1:1));
    poly1 = rotate(polyin,angle, origin1_at0');
    polyin2 = polyshape(link2_at0(1,:), link2_at0(2,:));
    angle2 = rad2deg(q(2:2)) + angle;
    poly2 = rotate(polyin2,angle2, origin1_at0');
    x_transl = robot.pivot2(1:1)*cos(q(1:1));
    y_transl = robot.pivot2(1:1)*sin(q(1:1));
    poly2 = translate(poly2,[x_transl y_transl]);
    pivot1 = [origin1_at0(1) origin1_at0(2)];
    pivot2 = [origin1_at0(1)+x_transl origin1_at0(2)+y_transl];
end