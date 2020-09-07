function [xtest]=rectsignal(x1,y1,x2,y2,w,Ts,T)
%P=(x,y)
% w pulse width

ttest=[0:Ts:T*2];
d1=[5:2*w:T*2];
d2=[10:2*w:T*2];

xtest=x2*pulstran(ttest,d1,'rectpuls',w);
ytest=y2*pulstran(ttest,d2,'rectpuls',w);
Ix=xtest==0;
Iy=ytest==0;
xtest=xtest+x1*Ix;
ytest=ytest+y1*Iy;
xtest=[ttest' xtest' zeros(size(xtest))' ytest' zeros(size(xtest))' zeros(size(xtest))' zeros(size(xtest))' zeros(size(xtest))' zeros(size(xtest))'];
% figure
% subplot(2,1,1)
% plot(xtest(:,1),xtest(:,2))
% subplot(2,1,2)
% plot(ttest,ytest)
% subplot(2,1,3)
% plot(xtest(:,2),xtest(:,4))
end