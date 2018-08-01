function plotPts(pts,mark_style,mark_size,mark_fore,mark_back)
    //Author : Maxens ACHIEPI
    //Space Robotics Laboratory - Tohoku University
    
    //Description:
    //Utility function for easier plotting
    
    //INPUT:
    
    //OUTPUT:
    
//----------------------------------------------------------------------------//
    
    if size(pts,2)==3 then
        plot3d(pts(:,1),pts(:,2),pts(:,3));
        h = get("hdl");
        h.surface_mode = "off";
        h.mark_mode = "on";
        h.mark_style = mark_style;
        h.mark_size_unit = "point";
        h.mark_size = mark_size;
        h.mark_foreground = mark_fore;
        h.mark_background = mark_back;
    elseif size(pts,2)==2 then
        plot(pts(:,1),pts(:,2),mark_style);
    end
    
endfunction
