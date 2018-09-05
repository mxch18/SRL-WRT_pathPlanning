-- attached to each leg tip dummy

function sysCall_init()

	prox_FL = sim.getObjectHandle("prox_FL") -- proximity sensor on leg tip
						 -- sensor range is 0.002m in FLont of leg tip
	dummy_tip_FL = sim.getObjectAssociatedWithScript(sim.handle_self)

	floor = sim.getScriptSimulationParameter(sim.handle_self,'floor') -- the floor's handle, defined in walker_base_init

	attached = false
	size = 0.01
	prox_FL_pos = {}
	prox_FL_mat = {}
	res_FL=-1
	dist_FL=0
	det_pt_FL={}
  newDum = -1
	
end

function sysCall_sensing()

	grip_FL = sim.getScriptSimulationParameter(sim.handle_self,'grip_FL') -- need to set this parameter in a walker_base child script

	res_FL,dist_FL,det_pt_FL = sim.checkProximitySensor(prox_FL,floor)

	if res_FL>0 then -- we detect the ground in FLont of the foot (less than 0.002m in FLont)
		if grip_FL then -- if gripping is activated
			if not attached then -- if we are not already gripping
				attached = true -- we are now
				-- get position of detected point in world FLame
				prox_FL_pos = sim.getObjectPosition(prox_FL,-1)
				prox_FL_mat = sim.getObjectMatrix(prox_FL,-1)
				det_pt_FL = sim.multiplyVector(prox_FL_mat,det_pt_FL)
				-- we create an invisible dummy in FLont of the leg tip
				newDum = sim.createDummy(size)
				sim.setObjectPosition(newDum,-1,det_pt_FL)
				sim.setObjectInt32Parameter(newDum,sim.objintparam_visibility_layer,256)
				sim.setObjectParent(newDum,floor,true)
				-- we connect
				sim.setLinkDummy(dummy_tip_FL,newDum)
				sim.setObjectInt32Parameter(dummy_tip_FL,sim.dummyintparam_link_type,0)
			end
		else -- gripping deactivated
			if attached then -- we are attached
                attached = false -- we are not anymore
				-- we disconnect
				sim.setLinkDummy(dummy_tip_FL,-1)
				-- we delete the dummy
				if newDum>0 then	
                    sim.removeObject(newDum) -- delete potential dummy
                    newDum = -1
                end
			end
		end
	end
end

function sysCall_cleanup()

	sim.setLinkDummy(dummy_tip_FL,-1) -- delete any remaining dummy link
	if newDum>0 then	
		sim.removeObject(newDum) -- delete potential dummy
	end

end
