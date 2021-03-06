-- this script is associated with the terrain shape (most probably low-poly heightfield shape for better performance)

function sysCall_init() -- this part will be executed one time just at the beginning of a simulation

    	-- init random seed
	-- math.randomseed(1234) -- not actually random for the moment

	-- adds a proximity sensor with range 100
	options = 1
	intParam = {0,0,0,0,0,0,0,0}
	floatParam = {0,100,0,0,0,0,0,0,0,0,0,0,0,0,0}
	color = nil
	proxSens = sim.createProximitySensor(sim.proximitysensor_ray_subtype, sim.objectspecialproperty_detectable_laser, options, intParam, floatParam, color)
	
	sim.setObjectName(proxSens,"pro_sensor") -- sets sensors name
	
	pro_sensor = sim.getObjectHandle("pro_sensor") -- gets sensor handle
	
	-- sets sensor orientation and position to origin
	sim.setObjectOrientation(pro_sensor,-1,{0,0,0})
	dec = 0.01
	sim.setObjectPosition(pro_sensor,-1,{0,0,-dec})

	-- terrain discretization
	-- we set the shape position. This means that:
	--	- the lowest point of the heightfield is at z=0 in world frame
	--	- center of the shape is at x=0 and y=0 in world frame

	heightfield = sim.getObjectAssociatedWithScript(sim_handle_self) -- gets terrain shape handle

	r,terrain_Z_max = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_max_z)
	r,terrain_Z_min = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_min_z)
	terrain_Z = terrain_Z_max - terrain_Z_min
	sim.setObjectPosition(heightfield,-1,{0,0,terrain_Z/2})

	-- gets terrain bounds

	r,terrain_X_max = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_max_x)
	r,terrain_X_min = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_min_x)
	terrain_X = terrain_X_max - terrain_X_min

	r,terrain_Y_max = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_max_y)
	r,terrain_Y_min = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_min_y)
	terrain_Y = terrain_Y_max-terrain_Y_min

	Kx = 40 -- number of grid points in x direction
	Ky = 40 -- number of grid points in y direction

	incX = terrain_X/Kx -- increment on X axis
	incY = terrain_Y/Ky -- increment on Y axis

	X = terrain_X_min + 0.001
	Y = terrain_Y_min + 0.001

	-- dummy size
	size = 0.05
	dum_tab = {} -- array of dummy handles
	k = 1
	
	-- create cost map

	for i = 1,Kx,1
	do
	-- explore all grid points and extract heights

		Y = terrain_Y_min + 0.001

		for j = 1,Ky,1
		do

			pro_pos = {X,Y,-dec}
			sim.setObjectPosition(pro_sensor,-1,pro_pos)

			result,distance,det_pt = sim.checkProximitySensor(pro_sensor,heightfield)

			if result>0 then
                distance = distance - dec
				dum_tab[k] = sim.createDummy(size)
                sim.setObjectInt32Parameter(dum_tab[k],sim.objintparam_visibility_layer,256)
				sim.setObjectPosition(dum_tab[k],-1,{X,Y,distance})
				sim.setObjectParent(dum_tab[k],heightfield,true)
				k = k+1
			end

			Y = Y + incY
			if Y > terrain_Y_max then Y = terrain_Y_max-0.001 end

		end

		X = X + incX
		if X > terrain_X_max then X = terrain_X_max-0.001 end

	end

end
