function [z]=lin_model(alpha,A,B,z0)

z = A*z0 + B*alpha;

end
