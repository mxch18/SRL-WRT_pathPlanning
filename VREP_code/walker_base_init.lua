function sysCall_init()

    sim.getScriptSimulationParameter(sim.handle_tree,'drawForce','true')

    sim.setScriptSimulationParameter(sim.handle_tree,'grip_FL','true')
    sim.setScriptSimulationParameter(sim.handle_tree,'grip_HL','true')
    sim.setScriptSimulationParameter(sim.handle_tree,'grip_FR','true')
    sim.setScriptSimulationParameter(sim.handle_tree,'grip_HR','true')
    
    -- TO MODIFY ACCORDING TO SCENE
    floor = sim.getObjectHandle("Plane");
    -- TO MODIFY ACCORDING TO SCENE

    sim.setScriptSimulationParameter(sim.handle_tree,'floor',floor);

end
