clear variables
close all
clc 

%% Create First robot
L(1) = Link('revolute','d',0,'a',0,'alpha',-pi/2);
L(2) = Link('revolute','d',0,'a',0,'alpha',+pi/2);
L(3) = Link('revolute','d',0.27,'a',0.085,'alpha',pi/2);
L(4) = Link('revolute','d',0,'a',0.06,'alpha',pi/2);
L(5) = Link('revolute','d',0.27,'a',0,'alpha',-pi/2);
L(6) = Link('revolute','d',0,'a',0,'alpha',pi/2);
L(7) = Link('revolute','d',0,'a',0,'alpha',0);

R1 = SerialLink(L,'name','SIA 5F','fast','1');

% Create Second robot
L(1) = Link('revolute','d',0,'a',0,'alpha',-pi/2);
L(2) = Link('revolute','d',0,'a',0,'alpha',pi/2);
L(3) = Link('revolute','d',.4,'a',0,'alpha',-pi/2);
L(4) = Link('revolute','d',0,'a',0,'alpha',pi/2);
L(5) = Link('revolute','d',.4,'a',0,'alpha',-pi/2);
L(6) = Link('revolute','d',0,'a',0,'alpha',pi/2);
L(7) = Link('revolute','d',.126,'a',0,'alpha',0);

R2 = SerialLink(L,'name','IIWA','fast','1');



