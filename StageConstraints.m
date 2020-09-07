function [Dt,Et,bt]=StageConstraints(A,B,D,cl,ch,ul,uh)

% your code goes here
[n,m]=size(B);
O=zeros(m,n);
Im=eye(m);

Dt=[D*A;-D*A;[O;O]];
Et=[D*B;-D*B;Im;-Im];
bt=[ch ;-cl ;uh ;-ul];
end

