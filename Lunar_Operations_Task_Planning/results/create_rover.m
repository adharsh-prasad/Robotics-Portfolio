function rover = create_rover()
    length = 0.5;
    width = 0.4;
    height = 0.2;
    
    vertices = [
        -length/2, -width/2, -height;
        length/2, -width/2, -height;
        length/2, width/2, -height;
        -length/2, width/2, -height;
        -length/2, -width/2, 0;
        length/2, -width/2, 0;
        length/2, width/2, 0;
        -length/2, width/2, 0
    ];
    
    faces = [
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8;
        1 2 3 4;
        5 6 7 8
    ];
    
    rover = struct('Vertices', vertices, 'Faces', faces);
end
