%% Returns a column vector with the values of the activation functions of the DMP
%  @param[in] dmp: The DMP object.
%  @param[in] x: phase variable
%  @param[out] Psi: column vector with the values of the activation functions of the DMP
function Psi = DMP_gaussian_kernel(dmp,x)

    n = length(x);
    Psi = zeros(dmp.N_kernels, n);
    
    for j=1:n
        Psi(:,j) = exp(-dmp.h.*((x(j)-dmp.c).^2));
    end   

end