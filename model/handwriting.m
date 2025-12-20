%% =========================================================================
%  STEP 1: USER CONFIGURATION
% =========================================================================
active_joints_indices = 1:11; 
active_cp_indices = [1, 2, 3]; 

WRIST_PIVOT = wrist_center + [0; -20; 0]; 

hand.qmin = -ones(hand.n, 1) * deg2rad(5); 
hand.qmax = ones(hand.n, 1) * deg2rad(90); 

n_total_joints = length(hand.q);
hand.qmin = -ones(n_total_joints, 1) * deg2rad(15); 
hand.qmax =  ones(n_total_joints, 1) * deg2rad(90);
if n_total_joints >= 1
    hand.qmin(1) = deg2rad(-45);
    hand.qmax(1) = deg2rad(45);
end

active_joints_indices = active_joints_indices(:); 
active_cp_indices = active_cp_indices(:);
active_cp_indices = active_cp_indices(1:3);
%% =========================================================================
%  STEP 2: CALIBRATION
% =========================================================================
q_home = hand.q(:); 
current_cp = hand.cp(1:3, :); 
active_cp_home = current_cp(:, active_cp_indices);

PEN_LENGTH_MM = 60;
PAPER_Z_OFFSET = 130;
TARGET_PEN_RADIUS = 4;

fingers_center = mean(active_cp_home, 2);

pen_tip_home = fingers_center - [0; 0; PEN_LENGTH_MM]; 

num_fingers = length(active_cp_indices);
tight_grip_offsets = zeros(3, num_fingers);
for i = 1:num_fingers
    radial_vec = active_cp_home(:, i) - fingers_center;
    r_vec_flat = radial_vec; 
    r_vec_flat(3) = 0; 
    current_radius = norm(r_vec_flat);
    
    if current_radius > 0

        shrink_ratio = TARGET_PEN_RADIUS / current_radius;

        tight_pos = fingers_center + (r_vec_flat * shrink_ratio);
        tight_pos(3) = active_cp_home(3, i);

        tight_grip_offsets(:, i) = tight_pos - pen_tip_home;
    else
        tight_grip_offsets(:, i) = active_cp_home(:, i) - pen_tip_home;
    end
end

grip_offsets = tight_grip_offsets;

target_width_mm = 10; 
raw_width = max(trajectory(:,1)) - min(trajectory(:,1));
if raw_width == 0, scale_factor = 1; else, scale_factor = target_width_mm / raw_width; end
trajectory = trajectory * scale_factor;

traj_center = mean(trajectory(:,1:2), 1);
target_xy = WRIST_PIVOT(1:2)'; 
shift = target_xy - traj_center;
trajectory(:,1) = trajectory(:,1) + shift(1);
trajectory(:,2) = trajectory(:,2) + shift(2);
trajectory(:,3) = WRIST_PIVOT(3) - PAPER_Z_OFFSET;

first_point = trajectory(1,:)';
pen_vec_home = WRIST_PIVOT - first_point; 
pen_axis_home = pen_vec_home / norm(pen_vec_home);
%% =========================================================================
%  STEP 3: SOLVER
% =========================================================================
disp('Solving...');
numFrames = size(trajectory, 1);
qm = zeros(length(q_home), numFrames);
qm(:,1) = q_home;

frozen_indices = setdiff(1:length(q_home), active_joints_indices);
q_frozen_vals = q_home(frozen_indices);
options = optimset('Display','off', 'TolX', 1e-4, 'TolFun', 1e-4);
sim_hand = hand; 
current_q_full = q_home;

for k = 1:numFrames

    target_tip = trajectory(k, :)';

    new_pen_vec = WRIST_PIVOT - target_tip;
    new_pen_axis = new_pen_vec / norm(new_pen_vec);

    v = cross(pen_axis_home, new_pen_axis);
    s = norm(v);
    c = dot(pen_axis_home, new_pen_axis);
    if s == 0, R = eye(3); else
        vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        R = eye(3) + vx + vx^2 * ((1-c)/s^2);
    end

    target_active_cp = repmat(target_tip, 1, 3) + (R * grip_offsets);

    q_active_start = current_q_full(active_joints_indices);
    q_active_start = q_active_start(:); 
    
    cost_func = @(q_sub) constrained_solver_step(q_sub, active_joints_indices, ...
                         current_q_full, sim_hand, target_active_cp, active_cp_indices);
    
    [best_q_active, ~] = fminsearch(cost_func, q_active_start, options);

    current_q_full(active_joints_indices) = best_q_active;
    current_q_full(frozen_indices) = q_frozen_vals; 
    
    qm(:, k) = current_q_full;
    
    if mod(k,10)==0, fprintf('.'); end
end
%% =========================================================================
%  STEP 4: ANIMATION
% =========================================================================
figure(2); clf;
view([-140 15]); axis equal; grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');

VISUAL_PEN_LEN = 120;
VISUAL_PEN_RADIUS = 5;
HALF_LEN = VISUAL_PEN_LEN / 2; 

[cyl_X, cyl_Y, cyl_Z] = cylinder(VISUAL_PEN_RADIUS, 12);
cyl_Z = cyl_Z * VISUAL_PEN_LEN - HALF_LEN;
for k = 1:numFrames
    clf; hold on; grid on; axis equal; view([-140 15]);

    if k > 1
        plot3(trajectory(1:k, 1), trajectory(1:k, 2), trajectory(1:k, 3), ...
              'r-', 'LineWidth', 1.5);
    end

    tip_pos = trajectory(k, :)';
    vec_to_wrist = WRIST_PIVOT - tip_pos;
    pen_dir = vec_to_wrist / norm(vec_to_wrist); 
    
    pen_center = tip_pos + (pen_dir * HALF_LEN);

    z_axis = pen_dir;
    x_axis = cross([0;1;0], z_axis); 
    if norm(x_axis) < 0.01, x_axis = cross([1;0;0], z_axis); end
    x_axis = x_axis / norm(x_axis);
    y_axis = cross(z_axis, x_axis);
    
    R = [x_axis, y_axis, z_axis];
    num_pts = numel(cyl_X);
    raw_pts = [cyl_X(:)'; cyl_Y(:)'; cyl_Z(:)'];
    rotated_pts = R * raw_pts;
    
    X_plot = reshape(rotated_pts(1,:) + pen_center(1), size(cyl_X));
    Y_plot = reshape(rotated_pts(2,:) + pen_center(2), size(cyl_Y));
    Z_plot = reshape(rotated_pts(3,:) + pen_center(3), size(cyl_Z));

    surf(X_plot, Y_plot, Z_plot, 'FaceColor', 'y', 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    plot3(tip_pos(1), tip_pos(2), tip_pos(3), 'k.', 'MarkerSize', 5);
    hand = SGmoveHand(hand, qm(:,k));
    SGplotHand(hand);       
    drawnow;
end
%% =========================================================================
%  HELPER FUNCTION
% =========================================================================
function err = constrained_solver_step(q_active, active_idx, q_template, hand_model, target_pts, active_cp_idx)

    q_template = q_template(:);
    q_active   = q_active(:);
    q_trial = q_template;
    q_trial(active_idx) = q_active;

    penalty_limits = 0;
    if isfield(hand_model, 'qmin') && isfield(hand_model, 'qmax')
        q_min = hand_model.qmin(:);
        q_max = hand_model.qmax(:);
        
        violation_min = max(0, q_min - q_trial);
        violation_max = max(0, q_trial - q_max);
        
        penalty_limits = 1e7 * sum(violation_min.^2 + violation_max.^2);
    end

    hand_model = SGmoveHand(hand_model, q_trial);
    full_cp = hand_model.cp(1:3, :);
    current_pts = full_cp(:, active_cp_idx);

    diff_vector = current_pts(:) - target_pts(:);
    tracking_error = 1e3 * sum(diff_vector.^2); 
    err = tracking_error + penalty_limits;
end