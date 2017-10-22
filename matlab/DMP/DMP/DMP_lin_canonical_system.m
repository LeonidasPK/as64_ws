%% DMP linear canonical system class
%  Implements a 1st order linearly decaying canonical system for a DMP.
%  The canonical system is defined as:
%     tau*dx = -a_x
%  where x is the phase variable and a_x the decay term and tau is scaling a 
%  factor defining the duration of the motion.
%

classdef DMP_lin_canonical_system < handle
   properties
       a_x % the decay factor of the phase variable
       tau % movement duration (can be used to scale temporally the motion)
   end
   
   methods
      %% initializes the canonical system
      %
      %  param[in] x_end: value of phase variable at the end of the motion
      %  param[in] tau: movement duration (can be used to scale temporally the motion)
      %
      %  param[out] can_sys: canonical system object
      %
      function can_sys = DMP_lin_canonical_system(x_end, tau)
          
          can_sys.init(x_end, tau);
          
      end 
      
      %% initializes the canonical system
      %
      %  param[in] x_end: value of phase variable at the end of the motion
      %  param[in] tau: movement duration (can be used to scale temporally the motion)
      %
      function init(can_sys, x_end, tau)
          
          can_sys.set_can_sys_params(x_end);
          can_sys.tau = tau;
      end
      
      %% sets the canonical system's time constants based on the value of the phase variable at the end of the movement
      %
      %  param[in] x_end: value of the phase variable at the end of the movement (t = tau)
      %
      function set_can_sys_params(can_sys, x_end)
          
          can_sys.a_x = 1-x_end;
          
      end

      %% Returns the derivative of the canonical system
      %
      %  param[in] x: current value of the phase variable
      %
      %  param[out] dx: derivative
      %
      function dx = get_derivative(can_sys, x)
          
           dx = -can_sys.a_x/can_sys.tau;

      end
      
      %% Returns the output of the canonical system for a continuous time interval
      %
      %  param[in] t: the time interval
      %  param[in] x0: initial value of the phase variable
      %
      %  param[out] x: the output of the canonical system for the time interval 't'
      %
      function x = get_continuous_output(can_sys, t, x0)
          
          x = x0 - can_sys.a_x*t/can_sys.tau;
          
          % to do
          % check if x < 0

      end

     
   end
end



