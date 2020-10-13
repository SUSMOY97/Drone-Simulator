function [ desired_state ] = traj_generator_set_time(t, state, waypoints)
persistent waypoints0 traj_time allCoeffs multFact time
if nargin > 2
    waypoints0 = waypoints';
    n = size(waypoints0, 1) - 1;
    multFact = 8;   
    time = 1;
    traj_time = 0:time:n*time; 
    A = zeros(multFact * n); 
    b = zeros(multFact * n, 3);
    numPts = size(waypoints0,1);
    b(1,:) = waypoints0(1,:);
    rowP = 1;
    for iPosPt = 2:numPts-1
        row  = rowP + 1;
        rowP = row + 1;        
        b(row,:)  = waypoints0(iPosPt,:);
        b(rowP,:) = waypoints0(iPosPt,:);
    end
    b(rowP+1,:) = waypoints0(end,:);
    
   
    posCon = [1 0 0 0 0 0 0 0;
        1 1 1 1 1 1 1 1];
    
    drvtvCon = [0 1 2 3 4 5 6 7;
        0 0 2 6 12 20 30 42;
        0 0 0 6 24 60 120 210];
    
    continuousCon = [0 1 2 3 4 5 6 7 0 -1 0 0 0 0 0 0;
        0 0 2 6 12 20 30 42 0 0 -2 0 0 0 0 0;
        0 0 0 6 24 60 120 210 0 0 0 -6 0 0 0 0;
        0 0 0 0 24 120 360 840 0 0 0 0 -24 0 0 0;
        0 0 0 0 0 120 720 2520 0 0 0 0 0 -120 0 0;
        0 0 0 0 0 0 720 5040 0 0 0 0 0 0 -720 0];
    
    rowEnd = 0;
    for iPos = 1:n
        colStart = (iPos-1)*multFact + 1;
        colEnd   = colStart + multFact - 1;
        rowStart = rowEnd + 1;
        rowEnd   = rowStart + 1;
        A(rowStart:rowEnd,colStart:colEnd) = posCon;
    end
    
    nxtRow = rowEnd;
    for iDrv = 1:3
        nxtRow = nxtRow + 1;
        A(nxtRow, iDrv+1) = 1;
    end
    
    startRow = (nxtRow + 1);
    endRow   = startRow + size(drvtvCon,1) - 1;
    startCol = (n-1) * multFact + 1;
    endCol   = size(A,2);
    
    A(startRow:endRow,startCol:endCol) = drvtvCon;
    
    colEnd = 0;
    for iCon = 1:n-1
        startRow = endRow + 1;
        endRow   = startRow + size(continuousCon,1) - 1;
        colStart = colEnd + 1;
        colEnd   = colStart + size(continuousCon,2) - 1;
        A(startRow:endRow,colStart:colEnd) = continuousCon;
        colEnd = multFact * iCon;
    end
    
    allCoeffs = A\b;
    
else
    if(t > traj_time(end))
        desired_state.pos = waypoints0(end,:)';
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
        t_index = find(traj_time <= t,1,'last');
        
        if(t == 0)
            desired_state.pos = waypoints0(1,:)';
            desired_state.vel = zeros(3,1);
            desired_state.acc = zeros(3,1);
        else
            if(t_index > 1)
                t = t - traj_time(t_index);
            end
            
            scale = t / time;
            
            coeffs = allCoeffs(((t_index-1)*multFact + 1): (t_index*8), :);
            desired_state.pos = ([1, scale, scale^2, scale^3, scale^4, scale^5, scale^6, scale^7] * coeffs)';
            desired_state.vel = ([0, 1, 2*scale, 3*scale^2, 4*scale^3, 5*scale^4, 6*scale^5, 7*scale^6] * coeffs)';
            desired_state.acc = ([0, 0, 2, 6*scale, 12*scale^2, 20*scale^3, 30*scale^4, 42*scale^5] * coeffs)';
        end
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end

