function fig = create_3D_lunar_base()
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
    temp_coord = Map_Start_Point + [Sr + Msib/2, Mcw/2 + Msil + new_sr*cos(asin(Msib/(2*new_sr)))];
    Superadobe_centers(1,:) = temp_coord;
    Superadobe_centers(2,:) = temp_coord + [Dbs,0];
    Superadobe_centers(3,:) = temp_coord + 2*[Dbs,0];

    %top right 2
    Superadobe_centers(4,:) = temp_coord + 2*[Dm,0] + 2*[Dbs,0];
    Superadobe_centers(5,:) = temp_coord + 2*[Dm,0] + 4*[Dbs,0];

    %bottom left 3
    new_sr = sqrt(Sr^2 - Msih^2);
    temp_coord = Map_Start_Point + [Sr + Msib/2, -(Mcw/2 + Msil + new_sr*cos(asin(Msib/(2*new_sr))))];
    Superadobe_centers(11,:) = temp_coord;
    Superadobe_centers(10,:) = temp_coord + 1*[Dbs,0];
    Superadobe_centers(9,:) = temp_coord + 2*[Dbs,0];

    %bottom right 3
    Superadobe_centers(8,:) = temp_coord + 2*[Dm,0] + 2*[Dbs,0];
    Superadobe_centers(7,:) = temp_coord + 2*[Dm,0] + 3*[Dbs,0];
    Superadobe_centers(6,:) = temp_coord + 2*[Dm,0] + 4*[Dbs,0];

    % Create domes
    connecting_paths = ['SA01_1', 'MA01', 'J11', 'J12', 'SA01_2', 'MA02', 'J12', 'J13';...
                        'SA02_1', 'MA02', 'J21', 'J22', 'SA02_2', 'MA03', 'J22', 'J23';...
                        'SA03_1', 'MA03', 'J31', 'J32', 'SA03_2', 'MA04', 'J32', 'J33';...
                        'SA04_1', 'MA05', 'J51', 'J52', 'SA04_2', 'MA06', 'J52', 'J53';...
                        'SA05_1', 'MA07', 'J71', 'J72', 'SA05_2', 'MA08', 'J72', 'J73';...
                        'SA06_1', 'MA09', 'J74', 'J73', 'SA06_2', 'MA08', 'J74', 'J71';...
                        'SA07_1', 'MA10', 'J64', 'J63', 'SA07_2', 'MA09', 'J64', 'J61';...
                        'SA08_1', 'MA11', 'J54', 'J53', 'SA08_2', 'MA10', 'J54', 'J51';...
                        'SA09_1', 'MA13', 'J34', 'J33', 'SA09_2', 'MA12', 'J34', 'J31';...
                        'SA10_1', 'MA14', 'J24', 'J23', 'SA10_2', 'MA13', 'J24', 'J21';...
                        'SA11_1', 'MA01', 'J14', 'J13', 'SA11_2', 'MA14', 'J14', 'J11'];
    n = 1;
    for i = 1:size(Superadobe_centers, 1)
        [coords, tangents] = create_dome(Superadobe_centers(i,1), Superadobe_centers(i,2), Sr, Msib, Msih, tw, y_axis, Chamber_Opacity, Path_Opacity);

        paths(connecting_paths(i,1:6)) = struct('connections', {{connecting_paths(i,11:13); connecting_paths(i,14:16); connecting_paths(i,7:10); connecting_paths(i,17:22)}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

        paths(connecting_paths(i,17:22)) = struct('connections', {{connecting_paths(i,27:29); connecting_paths(i,30:32); connecting_paths(i,23:26); connecting_paths(i,1:6)}}, ...
        'coordinates', coords.right(end:-1:1,:), ...    % Path points
        'tangents', tangents.right(end:-1:1,:), ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    end

    % Define superadobe centers in a more compact arrangement
    Intersection_centers = zeros(11,2);
    Map_Start_Point = [map_x_axis_offset, y_axis/2];

    %top left 3
    temp_coord = Map_Start_Point + [Sr, Mcw/2];
    Intersection_centers(1,:) = temp_coord;
    Intersection_centers(2,:) = temp_coord + [Dbs,0];
    Intersection_centers(3,:) = temp_coord + 2*[Dbs,0];

    %top right 2
    Intersection_centers(4,:) = temp_coord + 2*[Dm,0] + 2*[Dbs,0];
    Intersection_centers(5,:) = temp_coord + 2*[Dm,0] + 4*[Dbs,0];

    %bottom left 3
    temp_coord = Map_Start_Point + [Sr, -Mcw/2-Msil];
    Intersection_centers(11,:) = temp_coord;
    Intersection_centers(10,:) = temp_coord + 1*[Dbs,0];
    Intersection_centers(9,:) = temp_coord + 2*[Dbs,0];

    %top right 3
    Intersection_centers(8,:) = temp_coord + 2*[Dm,0] + 2*[Dbs,0];
    Intersection_centers(7,:) = temp_coord + 2*[Dm,0] + 3*[Dbs,0];
    Intersection_centers(6,:) = temp_coord + 2*[Dm,0] + 4*[Dbs,0];

    all_nodes = keys(paths);
    % Create intersections
    for i = 1:size(Intersection_centers, 1)
        [coords, tangents] = create_connecting_cube(Intersection_centers(i,:), 0, Msib, Msil, Msih, tw, Chamber_Opacity, Path_Opacity);
        if Intersection_centers(i,2) < (y_axis/2)
            coords.left = coords.left(end:-1:1,:);
            coords.right = coords.right(end:-1:1,:);
            tangents.left = tangents.left(end:-1:1,:);
            tangents.right = tangents.right(end:-1:1,:);
        end
        paths(all_nodes(2*i-1)).coordinates = [coords.right;paths(all_nodes(2*i-1)).coordinates];
        paths(all_nodes(2*i)).coordinates = [coords.left;paths(all_nodes(2*i)).coordinates];
        paths(all_nodes(2*i-1)).tangents = [tangents.right;paths(all_nodes(2*i-1)).tangents];
        paths(all_nodes(2*i)).tangents = [tangents.left;paths(all_nodes(2*i)).tangents];
    end    

    %payload area
    [coords, tangents] = payload_area(Map_Start_Point + [(map_x_length/2-Psm/2) -Mcw/2-Msil-Psm], 0, Psm, Psm, Psh, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('PA01_1') = struct('connections', {{'J44'; 'J41'; 'MA12'; 'PA01_2'}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    paths('PA01_2') = struct('connections', {{'J43'; 'J44'; 'MA11'; 'PA01_1'}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    Intersection_centers = temp_coord + [2*Dbs+Dm,0];
    [coords, tangents] = create_connecting_cube(Intersection_centers, 0, Msib, Msil, Msih, tw, Chamber_Opacity, Path_Opacity);
    coords.left = coords.left(end:-1:1,:);
    coords.right = coords.right(end:-1:1,:);
    tangents.left = tangents.left(end:-1:1,:);
    tangents.right = tangents.right(end:-1:1,:);
    paths('PA01_1').coordinates = [coords.right;paths('PA01_1').coordinates];
    paths('PA01_2').coordinates = [coords.left;paths('PA01_2').coordinates];
    paths('PA01_1').tangents = [tangents.right;paths('PA01_1').tangents];
    paths('PA01_2').tangents = [tangents.left;paths('PA01_2').tangents];

    % Create main path    
    [coords, tangents] = create_main_path(Map_Start_Point + [(Sr+Msib) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('MA02') = struct('connections', {{'SA01_2'; 'J12'; 'J13'; 'SA02_1'; 'J21'; 'J22'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    paths('MA14') = struct('connections', {{'SA11_2'; 'J14'; 'J13'; 'SA10_1'; 'J21'; 'J24'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = create_main_path(Map_Start_Point + [(Sr+Msib+Dbs) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('MA03') = struct('connections', {{'SA02_2'; 'J22'; 'J23'; 'SA03_1'; 'J31'; 'J32'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    paths('MA13') = struct('connections', {{'SA10_2'; 'J24'; 'J23'; 'SA9_1'; 'J31'; 'J34'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs) -Mcw/2], 0, Dm-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('MA04') = struct('connections', {{'SA03_2'; 'J32'; 'J33'; 'J41'; 'J42'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    paths('MA12') = struct('connections', {{'SA09_2'; 'J34'; 'J33'; 'PA01_1'; 'J41'; 'J44'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs+Dm) -Mcw/2], 0, Dm-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('MA05') = struct('connections', {{'SA04_2'; 'J42'; 'J43'; 'J51'; 'J52'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    paths('MA11') = struct('connections', {{'SA08_1'; 'J44'; 'J43'; 'PA01_2'; 'J51'; 'J54'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs+2*Dm) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('MA06') = struct('connections', {{'SA05_2'; 'J52'; 'J53'; 'SA06_2'; 'J61'; 'J62'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    paths('MA10') = struct('connections', {{'SA08_2'; 'J44'; 'J43'; 'SA07_1'; 'J51'; 'J54'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = create_main_path(Map_Start_Point + [(Sr+Msib+3*Dbs+2*Dm) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('MA07') = struct('connections', {{'J62'; 'J63'; 'SA05_1'; 'J71'; 'J72'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    paths('MA09') = struct('connections', {{'SA07_1'; 'J64'; 'J63'; 'SA06_1'; 'J71'; 'J74'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    % Create junctions
    [coords, tangents] = junction_path(Map_Start_Point + [Sr -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('J11') = struct('connections', {{'MA01'; 'SA01_1'; 'SA11_1'; 'J12'; 'J14'}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J12') = struct('connections', {{'SA01_1'; 'SA01_2'; 'J11'; 'J13'; 'MA01'; 'MA02'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J13') = struct('connections', {{'MA02'; 'MA14';'SA01_2'; 'SA11_2'; 'J12'; 'J14'}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J14') = struct('connections', {{'SA11_1'; 'SA11_2'; 'J11'; 'J13'; 'MA01'; 'MA14'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = junction_path(Map_Start_Point + [(Sr+Dbs) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('J21') = struct('connections', {{'MA02'; 'MA14'; 'SA02_1'; 'SA10_1'; 'J22'; 'J24'}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J22') = struct('connections', {{'SA02_1'; 'SA02_2'; 'J21'; 'J23'; 'MA02'; 'MA03'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J23') = struct('connections', {{'MA03'; 'MA13';'SA02_2'; 'SA10_2'; 'J22'; 'J24'}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J24') = struct('connections', {{'SA10_1'; 'SA10_2'; 'J21'; 'J23'; 'MA14'; 'MA13'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = junction_path(Map_Start_Point + [(Sr+2*Dbs) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('J31') = struct('connections', {{'MA03'; 'MA13'; 'SA03_1'; 'SA09_1'; 'J32'; 'J34'}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J32') = struct('connections', {{'SA03_1'; 'SA03_2'; 'J31'; 'J33'; 'MA03'; 'MA04'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J33') = struct('connections', {{'MA04'; 'MA12';'SA03_2'; 'SA09_2'; 'J32'; 'J34'}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J34') = struct('connections', {{'SA09_1'; 'SA09_2'; 'J31'; 'J33'; 'MA13'; 'MA12'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = junction_path(Map_Start_Point + [(Sr+2*Dbs+Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('J41') = struct('connections', {{'MA04'; 'MA12'; 'PA01_1'; 'J42'; 'J44'}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J42') = struct('connections', {{'J31'; 'J33'; 'MA04'; 'MA05'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J43') = struct('connections', {{'MA04'; 'MA12'; 'PA01_2'; 'J42'; 'J44'}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J44') = struct('connections', {{'PA01_1'; 'PA01_2'; 'J41'; 'J43'; 'MA12'; 'MA11'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = junction_path(Map_Start_Point + [(Sr+2*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('J51') = struct('connections', {{'MA05'; 'MA11'; 'SA04_1'; 'SA08_1'; 'J52'; 'J54'}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J52') = struct('connections', {{'SA04_1'; 'SA04_2'; 'J51'; 'J53'; 'MA05'; 'MA06'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J53') = struct('connections', {{'MA06'; 'MA10'; 'SA04_2'; 'SA08_2'; 'J52'; 'J54'}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J54') = struct('connections', {{'SA08_1'; 'SA08_2'; 'J51'; 'J53'; 'MA11'; 'MA10'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = junction_path(Map_Start_Point + [(Sr+3*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('J61') = struct('connections', {{'MA06'; 'MA10'; 'SA07_1'; 'J62'; 'J64'}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J62') = struct('connections', {{'J61'; 'J63'; 'MA06'; 'MA07'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J63') = struct('connections', {{'MA07'; 'MA09'; 'SA07_2'; 'J62'; 'J64'}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J64') = struct('connections', {{'SA07_1'; 'SA07_2'; 'J61'; 'J63'; 'MA10'; 'MA09'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    [coords, tangents] = junction_path(Map_Start_Point + [(Sr+4*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity);
    paths('J71') = struct('connections', {{'MA07'; 'MA09'; 'SA05_1'; 'J72'; 'J74'}}, ...
        'coordinates', coords.left, ...    % Path points
        'tangents', tangents.left, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J72') = struct('connections', {{'J71'; 'J73'; 'MA07'; 'MA08'}}, ...
        'coordinates', coords.top, ...    % Path points
        'tangents', tangents.top, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J73') = struct('connections', {{'MA08'; 'SA06_2';'SA06_2'; 'J72'; 'J74'}}, ...
        'coordinates', coords.right, ...    % Path points
        'tangents', tangents.right, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians
    paths('J74') = struct('connections', {{'SA06_1'; 'SA06_2'; 'J71'; 'J73'; 'MA08'; 'MA09'}}, ...
        'coordinates', coords.bottom, ...    % Path points
        'tangents', tangents.bottom, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    % Create the left most rail
    [coords, tangents] = create_main_path(Map_Start_Point - [0 Mcw/2], 0, Sr, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    x = Map_Start_Point(1);
    y = Map_Start_Point(2)-Mcw/2;
    z = 0;

    [X, Y] = meshgrid([x, x+tw], [y, y+Mcw]);
    Z_bottom = ones(size(X)) * (z + Msih-0.1);
    Z_top = ones(size(X)) * (z + Msih);
    h1 = mesh(X, Y, Z_bottom);
    h2 = mesh(X, Y, Z_top);

    temp_coord = y+tw/2:0.1: y+Mcw-tw/2;
    temp_tangent = repmat([0, 1, 0], length(temp_coord), 1);

    temp = [ones(size(temp_coord))'*(x+tw/2) temp_coord' ones(size(temp_coord))'*(z + Msih-0.1)];
    coords = [coords.bottom(end:-1:1,:); temp; coords.top];
    tangents = [tangents.bottom(end:-1:1,:); temp_tangent; tangents.top];
    paths('MA01') = struct('connections', {{'SA01_1'; 'SA11_1'; 'J12'; 'J14'; 'J11'}}, ...
        'coordinates', coords, ...    % Path points
        'tangents', tangents, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

    % Apply properties to all mesh objects
    meshes_structure = [h1; h2];

    % Create the right most rail
    [coords, tangents] = create_main_path(Map_Start_Point + [(Sr+Msib+4*Dbs+2*Dm) -Mcw/2], 0, Sr, Msib, Msih, tw, Chamber_Opacity, Path_Opacity);
    x = Map_Start_Point(1)+map_x_length;
    y = Map_Start_Point(2)-Mcw/2;
    z = 0;

    [X, Y] = meshgrid([x-tw, x], [y, y+Mcw]);
    Z_bottom = ones(size(X)) * (z + Msih-0.1);
    Z_top = ones(size(X)) * (z + Msih);
    h3 = mesh(X, Y, Z_bottom);
    h4 = mesh(X, Y, Z_top);

    temp_coord = y+tw/2:0.1: y+Mcw-tw/2;
    temp_tangent = repmat([0, 1, 0], length(temp_coord), 1);

    temp = [ones(size(temp_coord))'*(x+tw/2) temp_coord(end:-1:1,:)' ones(size(temp_coord))'*(z + Msih-0.1)];
    coords = [coords.top; temp; coords.bottom(end:-1:1,:)];
    tangents = [tangents.top; temp_tangent; tangents.bottom(end:-1:1,:)];
    paths('MA08') = struct('connections', {{'SA05_2'; 'SA06_2'; 'J72'; 'J74'; 'J13'}}, ...
        'coordinates', coords, ...    % Path points
        'tangents', tangents, ...       % Unit vectors for orientation
        'headings', [n,1]);          % Angles in radians

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
end