classdef MyRobot< handle
    properties
        Link_num = 7;
        Link_orgin
        Link_rot
        Joint
        Endeffector
        constant = 1;
        VIEW = [ 13 , 22 ]
    end

    methods
        % 讀取建模資料
        function this = MyRobot(path)
            for i = 1 : this.Link_num
                tpath = path + "\" + string(i-1) + ".stl";
                TR = stlread(tpath);
                this.Link_orgin{i}.Points = TR.Points*0.1*this.constant;
                this.Link_orgin{i}.Connect = TR.ConnectivityList;
                this.Link_rot{i}.Points = this.Link_orgin{i}.Points;
                this.Link_rot{i}.Connect = this.Link_orgin{i}.Connect;
                this.Endeffector = [];
            end
        end

        % 暫時畫圖工具
        function temp_Draw(this, link, color)
            link = link + 1;
            patch('Vertices', this.Link_rot{link}.Points, ...
                  'Faces', this.Link_rot{link}.Connect, ...
                  'FaceVertexCData', color, ...
                  'FaceColor', 'flat')
            view(3); axis vis3d; axis("equal"); grid on; 
            xlabel('x (cm)'); ylabel('y (cm)'); zlabel('z (cm)'); 
            hold on 

%             disp(link)
%             disp('min_x :')
%             disp(min(this.Link_rot{link}.Points(:, 1)))
%             disp('max_x :')
%             disp(max(this.Link_rot{link}.Points(:, 1)))
        end

        % 對初始建模做平移與旋轉
        function Link_RT(this, R, T, link)
            link = link + 1;
            alpha = deg2rad(R(:, 1));
            beta =  deg2rad(R(:, 2)); 
            gamma = deg2rad(R(:, 3));
            Rmatirx = [ cos(alpha).*cos(beta),   cos(alpha).*sin(beta).*sin(gamma) - sin(alpha).*cos(gamma), ... 
                                                          sin(alpha).*sin(gamma) + cos(alpha).*sin(beta).*cos(gamma);
                        sin(alpha).*cos(beta),   sin(alpha).*sin(beta).*sin(gamma) + cos(alpha).*cos(gamma), ... 
                                                         -cos(alpha).*sin(gamma) + sin(alpha).*sin(beta).*cos(gamma);
                       -sin(beta),               cos(beta).*sin(gamma),    cos(beta).*cos(gamma);];
            this.Link_rot{link}.Points = (Rmatirx*this.Link_orgin{link}.Points')';
            this.Link_rot{link}.Points = this.Link_rot{link}.Points + T;
        end

         % 順項運動學
        function Forward_Kinematic(this, Angle, Axis, DH)
            theta_DH = DH(:, 1);
            d_DH =     DH(:, 2);
            a_DH =     DH(:, 3);
            alpha_DH = DH(:, 4);
            Angle = deg2rad(Angle);
            T = eye(4);
            for i = 1 : Axis
                Ti = [cos(Angle(i) + theta_DH(i)), -sin(Angle(i) + theta_DH(i))*cos(alpha_DH(i)),  sin(Angle(i) + theta_DH(i))*sin(alpha_DH(i)), a_DH(i)*cos(Angle(i) + theta_DH(i));
                      sin(Angle(i) + theta_DH(i)),  cos(Angle(i) + theta_DH(i))*cos(alpha_DH(i)), -cos(Angle(i) + theta_DH(i))*sin(alpha_DH(i)), a_DH(i)*sin(Angle(i) + theta_DH(i));
                                                0,                              sin(alpha_DH(i)),                              cos(alpha_DH(i)),                             d_DH(i); 
                                                0,                                             0,                                             0,                                   1;];
                T = T * Ti;
            end    
            this.Joint.Pos{Axis} = T(1:3, 4);
            this.Joint.Dir{Axis} = T(1:3, 1:3);
        end

        % 回傳尤拉角旋轉矩陣
        function Output = Euler(Euler)
            alpha = Euler(:, 1);
            beta = Euler(:, 2); 
            gamma = Euler(:, 3);
            Output = [ cos(alpha).*cos(beta), cos(alpha).*sin(beta).*sin(gamma) - sin(alpha).*cos(gamma), ... 
                       sin(alpha).*sin(gamma) + cos(alpha).*sin(beta).*cos(gamma);
                       sin(alpha).*cos(beta), sin(alpha).*sin(beta).*sin(gamma) + cos(alpha).*cos(gamma), ... 
                      -cos(alpha).*sin(gamma) + sin(alpha).*sin(beta).*cos(gamma);
                      -sin(beta),    cos(beta).*sin(gamma),    cos(beta).*cos(gamma);];

        end

        % 對旋轉軸旋轉
        function Rot(this, Axis, Angle, direction, origin)

            this.Link_rot{Axis+1}.Points = MyRotate(this.Link_rot{Axis+1}.Points, direction, Angle, origin);
            
            function  newPoints = MyRotate(Points, direction, Angle, origin)
                % 罗德里格旋转公式
                v = Points - origin';
                direction = direction/norm(direction);
                k = [ones(1, length(v(:, 1)))*direction(1); ...
                     ones(1, length(v(:, 1)))*direction(2); ...
                     ones(1, length(v(:, 1)))*direction(3)];
                v_rot = v*cos(Angle) + (cross(k, v').*sin(Angle))' + (k.*dot(k, v')*(1 - cos(Angle)))';
                newPoints = origin' + v_rot; 
            end
        end
    
        % 正式畫圖
        function Draw(this, Angle)
            DH = DH_MDH('DH');
            % 取得各軸位置與座標方向
            for i = 1 : 6
                this.Forward_Kinematic(Angle, i, DH);
            end

            theta_DH = DH(:, 1);
            Angle = deg2rad(Angle)';

            figure(1)
            % 畫出各軸位置與座標方向
            plot3([0, 20], [0, 0], [0, 0], 'g', 'linewidth', 2); hold on
            plot3([0, 0], [0, 20], [0, 0], 'b', 'linewidth', 2); hold on
            plot3([0, 0], [0, 0], [0, 20], 'r', 'linewidth', 2); hold on
            view(3); axis vis3d; axis("equal"); grid on;
            xlabel('x (cm)'); ylabel('y (cm)'); zlabel('z (cm)'); view(this.VIEW)
            xlim([-60 60]); ylim([-30 30]); zlim([-10 90])
            for i = 1 : 6
                xbase = [this.Joint.Pos{i}, this.Joint.Pos{i} + 20 * this.Joint.Dir{i}(:, 1)];
                ybase = [this.Joint.Pos{i}, this.Joint.Pos{i} + 20 * this.Joint.Dir{i}(:, 2)];
                zbase = [this.Joint.Pos{i}, this.Joint.Pos{i} + 20 * this.Joint.Dir{i}(:, 3)];
                plot3(xbase(1, :), xbase(2, :), xbase(3, :), 'g', 'linewidth', 2); hold on
                plot3(ybase(1, :), ybase(2, :), ybase(3, :), 'b', 'linewidth', 2); hold on
                plot3(zbase(1, :), zbase(2, :), zbase(3, :), 'r', 'linewidth', 2); hold on
            end

            plot3([0, this.Joint.Pos{1}(1)], ...
                  [0, this.Joint.Pos{1}(2)], ...
                  [0, this.Joint.Pos{1}(3)], 'k', 'linewidth', 2); hold on
            for i = 1 : 5
                plot3([this.Joint.Pos{i}(1), this.Joint.Pos{i+1}(1)], ...
                      [this.Joint.Pos{i}(2), this.Joint.Pos{i+1}(2)], ...
                      [this.Joint.Pos{i}(3), this.Joint.Pos{i+1}(3)], 'k', 'linewidth', 2); hold on
            end

            this.Endeffector = [this.Endeffector; this.Joint.Pos{6}'];
            plot3(this.Endeffector(:, 1), this.Endeffector(:, 2), this.Endeffector(:, 3), 'r', 'linewidth', 2);

            this.Link_RT([0, 0, 0], [-3.6858, 0.0091, 12.6000], 0)
            this.temp_Draw(0, [255, 0, 0])
            
            this.Link_RT([0, 0, 90], [3.3000, -0.0709, 34.3500], 1)
            this.Rot(1, Angle(1), [0, 0, 1], [0, 0, 0]')
            this.temp_Draw(1, [0, 255, 0])
            
            this.Link_RT([180, -90, -90], [8.5261, 13.9291, 48], 2)
            this.Rot(2, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(2, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.temp_Draw(2, [255, 0, 0])
            
            this.Link_RT([180, -90, 0], [8.3000, -0.0709, 66.5], 3)
            this.Rot(3, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(3, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.Rot(3, Angle(3), this.Joint.Dir{2}(:, 3), this.Joint.Pos{2});
            this.temp_Draw(3, [0, 255, 0])
            
            this.Link_RT([90, 90, 0], [31.2849, -0.0709, 70.41], 4)
            this.Rot(4, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(4, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.Rot(4, Angle(3), this.Joint.Dir{2}(:, 3), this.Joint.Pos{2});
            this.Rot(4, Angle(4), this.Joint.Dir{3}(:, 3), this.Joint.Pos{3});
            this.temp_Draw(4, [255, 0, 0])
            
            this.Link_RT([0, 90, 0], [39.3000, -0.0709, 70.5], 5)
            this.Rot(5, Angle(1), [0, 0, 1], [0, 0, 0]');
            this.Rot(5, Angle(2), this.Joint.Dir{1}(:, 3), this.Joint.Pos{1});
            this.Rot(5, Angle(3), this.Joint.Dir{2}(:, 3), this.Joint.Pos{2});
            this.Rot(5, Angle(4), this.Joint.Dir{3}(:, 3), this.Joint.Pos{3});
            this.Rot(5, Angle(5), this.Joint.Dir{4}(:, 3), this.Joint.Pos{4});
            this.temp_Draw(5, [0, 255, 0])
        end
    end


end