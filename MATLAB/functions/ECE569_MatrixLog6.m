function expmat = ECE569_MatrixLog6(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a transformation matrix T in SE(3).
% Returns the corresponding se(3) representation of exponential 
% coordinates.
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% expmat = MatrixLog6(T)
% 
% Output:
% expc6 =
%         0         0         0         0
%         0         0   -1.5708    2.3562
%         0    1.5708         0    2.3562
%         0         0         0         0

[R, p] = ECE569_TransToRp(T);
omgmat = ECE569_MatrixLog3(R);
if isequal(omgmat, zeros(3))
    % expmat = ...
else
    % theta = acos((trace(R) - 1) / 2);
    % Note: equation (3.92) of MR is a bit confusing because of how they chose to normalize by theta.
    % You need to multiply (eqn. 3.92) by theta on both sides to properly implement the MatrixLog6 function. 
    % In summary, you should have v = (eqn. 3.92) * theta * p
    % expmat = ...
end
end