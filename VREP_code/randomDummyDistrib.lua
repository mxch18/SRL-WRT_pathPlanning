-- this script is associated with the terrain shape (most probably low-poly heightfield shape for better performance)

function sysCall_init() -- this part will be executed one time just at the beginning of a simulation

    	-- init random seed
	math.randomseed(1234) -- not actually random for the moment
	-- dummy size
	size = 0.05
	-- adds an invisible dummy
	-- we will move it around randomly in terrain bounds and then select closest point on the terrain.
	movDum = sim.createDummy(size)
	
	-- terrain discretization
	-- we assume that the shape has been correctly setup. This means that:
	--	- the lowest point of the heightfield is at z=0 in world frame
	--	- center of the shape is at x=0 and y=0 in world frame

	heightfield = sim.getObjectAssociatedWithScript(sim_handle_self) -- gets terrain shape handle
	
	-- gets terrain bounds
	r,terrain_X_max = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_max_x)
	r,terrain_X_min = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_min_x)
	terrain_X = terrain_X_max - terrain_X_min

	r,terrain_Y_max = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_max_y)
	r,terrain_Y_min = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_min_y)
	terrain_Y = terrain_Y_max-terrain_Y_min

	r,terrain_Z_max = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_max_z)
	r,terrain_Z_min = sim.getObjectFloatParameter(heightfield,sim.objfloatparam_objbbox_min_z)
	terrain_Z = terrain_Z_max-terrain_Z_min

	sim.setObjectPosition(heightfield,-1,{0,0,0})

	N = 1500 -- number of points on the terrain

	X = 0
	Y = 0
	Z = 0

	dum_tab = {} -- array of dummy handles

	for i = 1,N,1
	do
	-- move the dummy around and extract heights
		X = terrain_X_min + math.random()*terrain_X
		Y = terrain_Y_min + math.random()*terrain_Y
		Z = terrain_Z_min + math.random()*terrain_Z

		sim.setObjectPosition(movDum,-1,{X,Y,Z})
		r,seg = sim.checkDistance(movDum,heightfield,0)
		
		dum_tab[i] = sim.createDummy(size)
        sim.setObjectInt32Parameter(dum_tab[i],sim.objintparam_visibility_layer,256)
		sim.setObjectPosition(dum_tab[i],-1,{seg[4],seg[5],seg[6]})
		sim.setObjectParent(dum_tab[i],heightfield,true)

	end

end
