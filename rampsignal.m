function [xtest]=rampsignal(x1,y1,x2,y2,Ts,T)
%P=(x,y)
% w pulse width

T_shorter = T*2-4;
ttest=[0:Ts:T_shorter+5*Ts];
quarterperiod = T_shorter/8;
quartersamples = round(quarterperiod/Ts);

stepsizex = (x2-x1)/quartersamples;
stepsizey = (y2-y1)/quartersamples;
%xref1=[x1(1):stepsize(1):x2(1) zeros(1,quartersamples+1) ones(1,quartersamples+1)*x1(3) zeros(1,quartersamples+1) zeros(1,quartersamples+1) zeros(1,quartersamples+1) zeros(1,quartersamples+1) zeros(1,quartersamples+1)];
xref1 = [x1:stepsizex:x2]';
xref1 = [xref1 zeros(size(xref1)) ones(size(xref1))*y1 zeros(size(xref1)) zeros(size(xref1)) zeros(size(xref1)) zeros(size(xref1)) zeros(size(xref1))];

xref2=[y1:stepsizey:y2]';
xref2 = [ones(size(xref2))*x2 zeros(size(xref2)) xref2 zeros(size(xref2)) zeros(size(xref2)) zeros(size(xref2)) zeros(size(xref2)) zeros(size(xref2))];

xref3=flip(xref1(:,1));
xref3 = [xref3 zeros(size(xref3)) ones(size(xref3))*y2 zeros(size(xref3)) zeros(size(xref3)) zeros(size(xref3)) zeros(size(xref3)) zeros(size(xref3))];

xref4=flip(xref2(:,3));
xref4 = [ones(size(xref4))*x1 zeros(size(xref4)) xref4 zeros(size(xref4)) zeros(size(xref4)) zeros(size(xref4)) zeros(size(xref4)) zeros(size(xref4))];

xref = [xref1 ; xref2; xref3; xref4];
xref = [xref ; xref];
xtest = [ttest' xref];


%xtest=[ttest' xtest' zeros(size(xtest))' ytest' zeros(size(xtest))' zeros(size(xtest))' zeros(size(xtest))' zeros(size(xtest))' zeros(size(xtest))'];
% figure
% subplot(2,1,1)
% plot(xtest(:,1),xtest(:,2))
% subplot(2,1,2)
% plot(xtest(:,1),xtest(:,4))

end