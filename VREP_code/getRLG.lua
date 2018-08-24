function sysCall_init()

    filen = "/home/maxens/Stage/Code/git/SRL-WRT_pathPlanning/Scilab_code/robot_state.txt"
    walker_base = sim.getObjectAssociatedWithScript(sim_handle_self)
	
	size = 0.03

	dumFL = sim.createDummy(size)
	dumFR = sim.createDummy(size)
	dumHL = sim.createDummy(size)
	dumHR = sim.createDummy(size)
    
    joint_1_FL = sim.getObjectHandle('joint_1_FL')
    joint_2_FL = sim.getObjectHandle('joint_2_FL')
    joint_3_FL = sim.getObjectHandle('joint_3_FL')

    joint_1_FR = sim.getObjectHandle('joint_1_FR')
    joint_2_FR = sim.getObjectHandle('joint_2_FR')
    joint_3_FR = sim.getObjectHandle('joint_3_FR')
    
    joint_1_HL = sim.getObjectHandle('joint_1_HL')
    joint_2_HL = sim.getObjectHandle('joint_2_HL')
    joint_3_HL = sim.getObjectHandle('joint_3_HL')

    joint_1_HR = sim.getObjectHandle('joint_1_HR')
    joint_2_HR = sim.getObjectHandle('joint_2_HR')
    joint_3_HR = sim.getObjectHandle('joint_3_HR')

	file = io.open(filen,"r")
	
	p = {}
	m = {}
	angle_FR = {}
	angle_FL = {}
	angle_HR = {}
	angle_HL = {}

	os.setlocale("en_US.UTF-8") -- Lua too dumb to handle "." on FR locale... SMH

	-- first line is position
	for n in string.gmatch(file:read(),"%S+") do table.insert(p,n) end
	-- 2nd to 5th line is orientation matrix
	for n in string.gmatch(file:read(),"%S+") do table.insert(m,n) end
	for n in string.gmatch(file:read(),"%S+") do table.insert(m,n) end
	for n in string.gmatch(file:read(),"%S+") do table.insert(m,n) end
	-- next lines are for theta
	for n in string.gmatch(file:read(),"%S+") do table.insert(angle_HL,n) end
	for n in string.gmatch(file:read(),"%S+") do table.insert(angle_FL,n) end
	for n in string.gmatch(file:read(),"%S+") do table.insert(angle_HR,n) end
	for n in string.gmatch(file:read(),"%S+") do table.insert(angle_FR,n) end

	for i = 1,3,1
	do
		p[i] = tonumber(p[i])
		angle_FR[i] = tonumber(angle_FR[i])
		angle_FL[i] = tonumber(angle_FL[i])
		angle_HR[i] = tonumber(angle_HR[i])
		angle_HL[i] = tonumber(angle_HL[i])
	end

	for i = 1,9,1
	do
		m[i] = tonumber(m[i])
	end
	
	T_M = {m[1],m[2],m[3],p[1],m[4],m[5],m[6],p[2],m[7],m[8],m[9],p[3]}

	sim.setObjectMatrix(walker_base,-1,T_M)

	sim.setJointPosition(joint_1_FR,angle_FR[1])
	sim.setJointPosition(joint_2_FR,angle_FR[2])
	sim.setJointPosition(joint_3_FR,angle_FR[3])

	sim.setJointPosition(joint_1_FL,angle_FL[1])
	sim.setJointPosition(joint_2_FL,angle_FL[2])
	sim.setJointPosition(joint_3_FL,angle_FL[3])

	sim.setJointPosition(joint_1_HR,angle_HR[1])
	sim.setJointPosition(joint_2_HR,angle_HR[2])
	sim.setJointPosition(joint_3_HR,angle_HR[3])

	sim.setJointPosition(joint_1_HL,angle_HL[1])
	sim.setJointPosition(joint_2_HL,angle_HL[2])
	sim.setJointPosition(joint_3_HL,angle_HL[3])

	file:close()

end

function sysCall_cleanup()

end

