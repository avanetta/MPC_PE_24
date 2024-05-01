%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H, h] = lqr_maxPI(Q, R, params)
    
    ctrl = LQR(Q,R,params);
    K = ctrl.K;
    %define Autonomous system according to LQR law:
    A_LQR = params.model.A - params.model.B*K; 
    
    system = LTISystem('A', A_LQR);
    %constraints are not found in params, even though they should be
    %there???
    %{
    system.x.min = [params.constraints.T1Min; params.constraints.T2Min; params.constraints.T3Min];
    system.x.max = [params.constraints.T1Max; params.constraints.T2Max; params.constraints.T3Max];
    system.u.min = [params.constraints.P1Min; params.constraints.P2Min];
    system.u.max = [params.constraints.P1Max; params.constraints.P2Max];
    %}
    InvSet = system.invariantSet()
    InvSet.plot()
    
    
end

