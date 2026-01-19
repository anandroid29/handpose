hand = SGparadigmatic;

hand = SGhumanWritingConf(hand);

hand = SGaddFtipContact(hand,1,1:2);
hand = SGaddContact(hand,1,3,3,1);

[hand,object] = SGmakeObject(hand);

object.center(1) = object.center(1)+12;
object.center(3) = object.center(3)-30;
object.center(2) = object.center(2)+10;

%%
% figure(2)
% view([-144 10]);
% 
% hold on
% plot3(object.center(1),object.center(2),object.center(3),'rd','LineWidth',3,'MarkerSize',8)
% hold on
% grid on
% for i = 1:size(hand.cp,2)
%     plot3(hand.cp(1,i),hand.cp(2,i),hand.cp(3,i),'m*','Linewidth',2,'MarkerSize',8)
%     quiver3(hand.cp(1,i),hand.cp(2,i),hand.cp(3,i),object.normals(1,i),object.normals(2,i),object.normals(3,i),10,'LineWidth',2)
% end
% axis('equal')
% out = SGdefinePencil(object);
% SGplotHand(hand)
