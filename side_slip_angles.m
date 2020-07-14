function side_slips =side_slip_angles(X,data)
  
  %extract_parameters;
%    g   = data.gravity;
%   M   = data.mass;
%   Iz  = data.Iz;
  Lr  = data.Lr;
  Lf  = data.Lf;
  kv  = data.kv;
  %
%   Byr = data.Byr;
%   Cyr = data.Cyr;
%   Dyr = data.Dyr;
%   Eyr = data.Eyr;
%   Kyr = data.Kyr;
%   
%   Byf = data.Byf;
%   Cyf = data.Cyf;
%   Dyf = data.Dyf;
%   Eyf = data.Eyf;
%   Kyf = data.Kyf;
%   
  side_slips = zeros(2,1);
  t1 = X(6);
  t3 = X(5);
  t6 = 0.1e1 / X(4);
  side_slips(1) = -(t1 * Lf + t3) * t6;
  side_slips(2) = U(3) - (-t1 * Lr + t3) * t6;
end
