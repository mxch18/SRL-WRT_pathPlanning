function plotPts(pts,mark_style,mark_size,mark_fore,mark_back)
    plot3d(pts(:,1),pts(:,2),pts(:,3));
    h = get("hdl");
    h.surface_mode = "off";
    h.mark_mode = "on";
    h.mark_style = mark_style;
    h.mark_size_unit = "point";
    h.mark_size = mark_size;
    h.mark_foreground = mark_fore;
    h.mark_background = mark_back;
endfunction
