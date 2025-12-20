% figure(99); clf;
% SGplotHand(hand); 
% hold on; grid on; axis equal; view(3);

base_positions = [];
for i = 1:hand.n
    base_positions = [base_positions, hand.F{i}.base(1:3, 4)];
end
wrist_center = mean(base_positions, 2);

%% Uncomment to Visualise the Palm Vectors
% plot3(wrist_center(1), wrist_center(2), wrist_center(3), 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Base Center');
% 
% quiver3(wrist_center(1), wrist_center(2), wrist_center(3), 30, 0, 0, 'r', 'LineWidth', 2, 'DisplayName', 'X (Red)');
% quiver3(wrist_center(1), wrist_center(2), wrist_center(3), 0, 30, 0, 'g', 'LineWidth', 2, 'DisplayName', 'Y (Green)');
% quiver3(wrist_center(1), wrist_center(2), wrist_center(3), 0, 0, 30, 'b', 'LineWidth', 2, 'DisplayName', 'Z (Blue)');
% 
% WRIST_PIVOT = wrist_center + [0; 40; 20];
% legend show;

