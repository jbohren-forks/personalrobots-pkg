function[] = genmprim(outfilename)

%
%generates motion primitives and saves them into file
%
%written by Maxim Likhachev
%---------------------------------------------------
%

%defines

LINESEGMENT_MPRIMS = 1; %set the desired type of motion primitives
UNICYCLE_MPRIMS = 0;



if LINESEGMENT_MPRIMS == 1
    resolution = 0.01;
    numberofangles = 32; %preferably a power of 2
    numberofprimsperangle = 14;
    basemprimendpts_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts_c(1,:) = [1 0 0 1];
    basemprimendpts_c(2,:) = [4 0 0 1];
    basemprimendpts_c(3,:) = [8 0 0 1];
    basemprimendpts_c(4,:) = [6 2 0 1];
    basemprimendpts_c(5,:) = [6 -2 0 1];
    basemprimendpts_c(6,:) = [2 3 0 1];
    basemprimendpts_c(7,:) = [2 -3 0 1];
    basemprimendpts_c(8,:) = [-5 0 0 5];
    %1/32 theta change
    basemprimendpts_c(9,:) = [6 2 1 1];
    basemprimendpts_c(10,:) = [6 -2 -1 1];
    %2/32 theta change
    basemprimendpts_c(11,:) = [4 3 2 1];
    basemprimendpts_c(12,:) = [4 -3 -2 1];
    %turn in place
    basemprimendpts_c(13,:) = [0 0 1 1];
    basemprimendpts_c(14,:) = [0 0 -1 1];
    basemprimendpts_c(15,:) = [0 0 3 1];
    basemprimendpts_c(16,:) = [0 0 -3 1];    
elseif UNICYCLE_MPRIMS == 1
    fprintf(1, 'ERROR: unsupported mprims type\n');
    return;
else
    fprintf(1, 'ERROR: undefined mprims type\n');
    return;    
end;
    
    


fout = fopen(outfilename, 'w');


%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

%iterate over angles
for angleind = 1:numberofangles
    
    figure(1);
    hold off;
    
    %iterate over primitives    
    for primind = 1:numberofprimsperangle
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);
        
        %now figure out what action will be        
        baseendpose_c = basemprimendpts_c(primind,1:3);
        additionalactioncostmult = basemprimendpts_c(primind,4);
        
        %rotate by the start angle
        angle = (angleind-1)*2*pi/numberofangles;
        endx_c = round(baseendpose_c(1)*cos(angle)) + ceil(abs(baseendpose_c(2)*sin(angle)))*sign(-baseendpose_c(2)*sin(angle));
        endy_c = round(baseendpose_c(1)*sin(angle)) + ceil(abs(baseendpose_c(2)*cos(angle)))*sign(baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];        
        
        %generate intermediate poses (remember they are w.r.t 0,0 (and not
        %centers of the cells)
        numofsamples = 10;
        intermcells_m = zeros(numofsamples,3);
        if LINESEGMENT_MPRIMS == 1
            startpt = [0 0 angle];
            endpt = [endpose_c(1)*resolution endpose_c(2)*resolution (angleind - 1 + baseendpose_c(3))*2*pi/numberofangles];
            intermcells_m = zeros(numofsamples,3);
            for iind = 1:numofsamples
                intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                                        startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                                        rem(startpt(3) + (endpt(3) - startpt(3))*(iind-1)/(numofsamples-1), 2*pi)];
            end;            
        end;
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        plot(intermcells_m(:,1), intermcells_m(:,2));
        text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        hold on;
        
    end;
    grid;
    pause;
end;
        
fclose('all');