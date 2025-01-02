% function fig = create_3D_lunar_base()
    close all
    addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\structures')

    % Define base parameters
    Sr = 4;%Superadobe_radius
    So = 2;%Superadobe_offset
    Dbs = 2*Sr + So;%Distance_btw_superadobe
    Mcel = 2;%Main_chamber_end_length
    Mcw = 2;%Main_chamber_width
    Msil = 4;%Mainpath_Superadobe_intersection_length
    Msib = 2;%Mainpath_Superadobe_intersection_breadth
    Msih = 2;%Mainpath_Superadobe_intersection_height
    Sd = 4;%Supplies_dim
    Psm = 6;%Payload area dimensions
    Psh = 3;%Payload area height
    Dm = 1/2*Psm + Sr + So;%Distance_in_middle
    tw = 0.3; %Track width

    %Axis dimensions
    map_x_length = 2*(Sr + 2*Dbs + 1*Dm + Msib/2);
    map_y_length = (Sr*4 + Msil*2 + Mcw);
    x_axis = 5*(floor(map_x_length/5) + 1);
    y_axis = 5*(floor(map_y_length/5) + 1);
    z_axis = 5*(floor(Psm/5) + 1);
    map_x_axis_offset = (x_axis - map_x_length)/2;
    map_y_axis_offset = (y_axis - map_y_length)/2;

    % Path variables 
    paths = dictionary();    

    % Create fullscreen figure with black background
    fig = figure('Units', 'normalized', 'Position', [0 0 1 1], ...
                'Color', 'black', 'MenuBar', 'none', ...
                'ToolBar', 'none', 'WindowState', 'fullscreen');
    ax = axes('Color', 'black', 'Position', [0 0 1 1]);
    hold on;
    Chamber_Opacity = 0.3;
    Path_Opacity = 0.4;
    
    % Set view for top-down perspective with slight angle
    view(45, 45);
   
    campos([x_axis+50 -y_axis 50]);
    camtarget([x_axis/2 y_axis/2 0]);

    % Set up main lighting
    lighting gouraud;
    camlight('headlight');

    % Add additional lights for better depth perception
    camlight('right');
    camlight('left');

    % Customize light properties
    h = findobj(gca,'Type','light');
    set(h,'Style','infinite');
    set(h,'Color',[1 1 1]); % White light

    
    % Define superadobe centers in a more compact arrangement
    Superadobe_centers = zeros(11,2);
    Map_Start_Point = [map_x_axis_offset, y_axis/2];
    
    %top left 3
    new_sr = sqrt(Sr^2 - Msih^2);
    temp = Map_Start_Point + [Sr + Msib/2, Mcw/2 + Msil + new_sr*cos(asin(Msib/(2*new_sr)))];
    Superadobe_centers(1,:) = temp;
    Superadobe_centers(2,:) = temp + [Dbs,0];
    Superadobe_centers(3,:) = temp + 2*[Dbs,0];

    %top right 2
    Superadobe_centers(4,:) = temp + 2*[Dm,0] + 2*[Dbs,0];
    Superadobe_centers(5,:) = temp + 2*[Dm,0] + 4*[Dbs,0];

    %bottom left 3
    new_sr = sqrt(Sr^2 - Msih^2);
    temp = Map_Start_Point + [Sr + Msib/2, -(Mcw/2 + Msil + new_sr*cos(asin(Msib/(2*new_sr))))];
    Superadobe_centers(11,:) = temp;
    Superadobe_centers(10,:) = temp + 1*[Dbs,0];
    Superadobe_centers(9,:) = temp + 2*[Dbs,0];

    %bottom right 3
    Superadobe_centers(8,:) = temp + 2*[Dm,0] + 2*[Dbs,0];
    Superadobe_centers(7,:) = temp + 2*[Dm,0] + 3*[Dbs,0];
    Superadobe_centers(6,:) = temp + 2*[Dm,0] + 4*[Dbs,0];

    % Create domes
    connecting_paths = ['SA01_1', 'MA01', 'J11', 'J12', 'SA01_2', 'MA02', 'J12', 'J13';...
                        'SA02_1', 'MA02', 'J21', 'J22', 'SA02_2', 'MA03', 'J22', 'J23';...
                        'SA03_1', 'MA03', 'J31', 'J32', 'SA03_2', 'MA04', 'J32', 'J33';...
                        'SA04_1', 'MA05', 'J51', 'J52', 'SA04_2', 'MA06', 'J52', 'J53';...
                        'SA05_1', 'MA07', 'J71', 'J72', 'SA05_2', 'MA08', 'J72', 'J73';...
                        'SA06_1', 'MA09', 'J74', 'J73', 'SA06_2', 'MA08', 'J74', 'J71';...
                        'SA07_1', 'MA10', 'J64', 'J63', 'SA07_2', 'MA09', 'J64', 'J61';...
                        'SA08_1', 'MA11', 'J54', 'J53', 'SA08_2', 'MA10', 'J54', 'J51';...
                        'SA09_1', 'MA13', 'J34', 'J33', 'SA09_2', 'MA14', 'J34', 'J31';...
                        'SA10_1', 'MA14', 'J24', 'J23', 'SA10_2', 'MA12', 'J24', 'J21';...
                        'SA11_1', 'MA01', 'J14', 'J13', 'SA11_2', 'MA14', 'J14', 'J11'];
    n = 1;
    for i = 1:size(Superadobe_centers, 1)
        [coords, tangents] = create_dome(Superadobe_centers(i,1), Superadobe_centers(i,2), Sr, Msib, Msih, tw, y_axis, Chamber_Opacity, Path_Opacity);
        
        paths(connecting_paths(i,1:6)) = struct('connections', {{connecting_paths(i,11:13); connecting_paths(i,14:16); connecting_paths(i,7:10)}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

        paths(connecting_paths(i,17:22)) = struct('connections', {{connecting_paths(i,27:29); connecting_paths(i,30:32); connecting_paths(i,23:26)}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    end

    % Define superadobe centers in a more compact arrangement
    Intersection_centers = zeros(12,2);
    Map_Start_Point = [map_x_axis_offset, y_axis/2];

    %top left 3
    temp = Map_Start_Point + [Sr, Mcw/2];
    Intersection_centers(1,:) = temp;
    Intersection_centers(2,:) = temp + [Dbs,0];
    Intersection_centers(3,:) = temp + 2*[Dbs,0];

    %top right 2
    Intersection_centers(4,:) = temp + 2*[Dm,0] + 2*[Dbs,0];
    Intersection_centers(5,:) = temp + 2*[Dm,0] + 4*[Dbs,0];

    %bottom left 3
    temp = Map_Start_Point + [Sr, -Mcw/2-Msil];
    Intersection_centers(11,:) = temp;
    Intersection_centers(10,:) = temp + 1*[Dbs,0];
    Intersection_centers(9,:) = temp + 2*[Dbs,0];

    %top right 3
    Intersection_centers(8,:) = temp + 2*[Dm,0] + 2*[Dbs,0];
    Intersection_centers(7,:) = temp + 2*[Dm,0] + 3*[Dbs,0];
    Intersection_centers(6,:) = temp + 2*[Dm,0] + 4*[Dbs,0];

    %payload area
    payload_area(Map_Start_Point + [(map_x_length/2-Psm/2) -Mcw/2-Msil-Psm], 0, Psm, Psm, Psh, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    Intersection_centers(12,:) = [Intersection_centers;temp + [2*Dbs+Dm,0]];

    % Create intersections
    for i = 1:size(Intersection_centers, 1)
        create_connecting_cube(Intersection_centers(i,:), 0, Msib, Msil, Msih, tw, Chamber_Opacity, Path_Opacity)
    end

    % Create main path
    create_main_path(Map_Start_Point - [0 Mcw/2], 0, Sr, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+Dbs) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs) -Mcw/2], 0, Dm-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs+Dm) -Mcw/2], 0, Dm-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs+2*Dm) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+3*Dbs+2*Dm) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+4*Dbs+2*Dm) -Mcw/2], 0, Sr, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)

    % Create junctions
    junction_path(Map_Start_Point + [Sr -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+Dbs) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+2*Dbs) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+2*Dbs+Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+2*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+3*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+4*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)


    % Create the left most rail
    x = Map_Start_Point(1);
    y = Map_Start_Point(2)-Mcw/2;
    z = 0;

    [X, Y] = meshgrid([x, x+tw], [y, y+Mcw]);
    Z_bottom = ones(size(X)) * (z + Msih-0.1);
    Z_top = ones(size(X)) * (z + Msih);
    h1 = mesh(X, Y, Z_bottom);
    h2 = mesh(X, Y, Z_top);

    % Apply properties to all mesh objects
    meshes_structure = [h1; h2];

    % Create the left most rail
    x = Map_Start_Point(1)+map_x_length;
    y = Map_Start_Point(2)-Mcw/2;
    z = 0;

    [X, Y] = meshgrid([x-tw, x], [y, y+Mcw]);
    Z_bottom = ones(size(X)) * (z + Msih-0.1);
    Z_top = ones(size(X)) * (z + Msih);
    h3 = mesh(X, Y, Z_bottom);
    h4 = mesh(X, Y, Z_top);

    % Apply properties to all mesh objects
    meshes_structure = [meshes_structure; h3; h4];

    for h = meshes_structure'
        set(h, 'FaceColor', [1 1 1], ...
               'EdgeColor', 'none', ...
               'FaceAlpha', Chamber_Opacity, ...
               'FaceLighting', 'gouraud', ...
               'AmbientStrength', 0.3, ...
               'DiffuseStrength', 0.8, ...
               'SpecularStrength', 0.9, ...
               'SpecularExponent', 25);
    end

    % Set axis properties
    axis equal;
    axis([0 x_axis 0 y_axis 0 z_axis]);
    grid off;
    set(gca, 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');
    fig = gcf;
% end