%% This function creates a saved Matlab workspace that includes the angles
%  achieved with different plications as specified in Gerber et al. (2003).
%  The structure includes values for the mean, and lowest and highest range
%  values achieved in the study.
%
%  This code should be run from within the 'Supplementary' folder to
%  generate the database in the appropriate location.
%
%  REFERENCES
%  Gerber et al. (2003). Effect of selective capsulorrhaphy on the passive
%  range of motion of the glenohumeral joint. J Bone Joint Surg, 85A(1):
%  48-55.

%% Create database

%Navigate to appropriate directory to store data
cd('..\..\SupportingData');

%Generate the data structures

%Plication labels
plications = [{'None'}; {'Anterosuperior'}; {'Anteroinferior'};
    {'TotalAnterior'}; {'Posterosuperior'}; {'Posteroinferior'};
    {'TotalPosterior'}; {'TotalSuperior'}; {'TotalInferior'}];

%Angles
angles = [{'Abduction'}; {'Flexion'}; {'ExtRot0'};
    {'IntRot0'}; {'ExtRot45'}; {'IntRot45'};
    {'ExtRot90'}; {'IntRot90'}];

%Generate data table matching order of plications and angles for the mean,
%lowest and highest values achieved. Note that elevation angles (i.e.
%abduction and flexion) are always positive; while external and internal
%rotation angles are represented as negative and positive, respectively. In
%some cases internal rotation will be listed as negative, seemingly because
%the internal rotation torque was overcome by external rotation resistance
%from the joint capsule.
meanAngles = [91.5, 85.6, -53.4, 44.6, -104.4, 39.0, -133.0, 30.8;
    89.8, 71.8, -23.3, 44.1, -85.8, 36.4, -123.9, 26.9;
    72.1, 70.5, -32.8, 44.1, -69.3, 36.1, -87.3, 23.1;
    73.9, 66.3, -21.3, 42.8, -67.3, 35.3, -86.1, 21.4;
    82.1, 76.8, -48.9, 28.5, -103.4, 28.3, -130.8, 27.5;
    80.0, 71.5, -49.6, 35.6, -104.0, 21.4, -131.1, 10.3;
    76.6, 66.5, -49.1, 23.1, -107.8, 11.8, -129.5, 9.8;
    90.0, 68.3, -17.8, 29.0, -82.3, 27.8, -115.5, 24.6;
    63.8, 65.8, -32.8, 37.5, -74.6, 19.2, -87.3, 4.0];

lowestAngles = [77, 68, -29, 26, -80, 15, -115, 7;
    73, 55, -5, 23, -55, 10, -90, 2;
    50, 58, -10, 22, -35, 18, -26, -4;
    46, 50, -4, 25, -33, 19, -20, -3;
    43, 58, -30, 5, -90, 10, -116, 5;
    62, 62, -32, 13, -92, 4, -112, -12;
    46, 40, -22, 3, -92, -21, -117, 22;
    72, 50, -4, 9, -57, 10, -90, 8;
    48, 56, -10, 12, -45, 0, -25, -22];

highestAngles = [122, 121, -75, 64, -122, 81, -162, 79;
    115, 88, -52, 74, -128, 81, -162, 79;
    103, 91, -53, 62, -94, 64, -106, 63;
    103, 85, -34, 63, -96, 63, -125, 48;
    119, 97, -75, 54, -112, 52, -149, 66;
    88, 88, -75, 59, -130, 34, -164, 32;
    99, 85, -75, 51, -130, 31, -144, 40;
    120, 93, -46, 46, -98, 47, -140, 49;
    87, 85, -56, 60, -86, 32, -112, 30];

%% Save database

save('Gerber2003_AnglesDatabase.mat','angles','plications',...
    'meanAngles','lowestAngles','highestAngles');

%%%%% ----- End of createGerber2003_anglesDatabase.m ----- %%%%%