%% Magic Formula (Normalized)
%  Compute the pure longitudinal/lateral normalized force

function F = magic_formula(kappa,B,C,D,E)
  
  F = D*sin(C*atan(kappa*B-(kappa*B-atan(kappa*B))*E));

end