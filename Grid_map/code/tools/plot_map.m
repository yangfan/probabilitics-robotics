function plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t, window)

  clf;
  hold on;
  axis tight;
  axis(mapBox);
  map = map';
  imshow(ones(size(map)) - log_odds_to_prob(map))

  s = size(map)(1:2); 
  set(gcf, "position", [50 50 5*s]) 
  set(gca, "position", [.05 .05 .9 .9]) 
	traj = [poses(1:t,1)';poses(1:t,2)'];
	traj = world_to_map_coordinates(traj, gridSize, offset);
  
	plot(traj(1,:),traj(2,:),'g')
	plot(robPoseMapFrame(1),robPoseMapFrame(2),'bo','markersize',6,'linewidth',3)
	plot(laserEndPntsMapFrame(1,:),laserEndPntsMapFrame(2,:),'ro','markersize',4)
  hold off;

  if window
    figure(1, "visible", "on");
    drawnow;
    pause(0.1);
  else
    figure(1, "visible", "off");
    filename = sprintf('../plots/gridmap_s1_%03d.png', t);
    print(filename, '-dpng');
  endif

  
  end
