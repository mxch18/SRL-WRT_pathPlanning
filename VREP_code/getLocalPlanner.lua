function sysCall_init()

  -- ADD WALKER BASE INIT HERE --
    
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

	angles_joints = {}

	file_name = "/home/maxens/Stage/Code/git/SRL-WRT_pathPlanning/Scilab_code/leg_theta.txt"

	i = 1
	i_max = 0

	os.setlocale("en_US.UTF-8") -- Lua too dumb to handle "." on FR locale... SMH

	-- read all file
	for line in io.lines(file_name) do
		for n in string.gmatch(line,"%S+") do table.insert(angles_joints,tonumber(n)) end
		i_max = i_max + 1
	end

	i_max = i_max*12

end

function sysCall_cleanup()

end

function sysCall_actuation()

	if (i+11) < i_max then
		sim.setJointTargetPosition(joint_1_HL,angles_joints[i])
		sim.setJointTargetPosition(joint_2_HL,angles_joints[i+1])
		sim.setJointTargetPosition(joint_3_HL,angles_joints[i+2])

		sim.setJointTargetPosition(joint_1_FL,angles_joints[i+3])
		sim.setJointTargetPosition(joint_2_FL,angles_joints[i+4])
		sim.setJointTargetPosition(joint_3_FL,angles_joints[i+5])

		sim.setJointTargetPosition(joint_1_HR,angles_joints[i+6])
		sim.setJointTargetPosition(joint_2_HR,angles_joints[i+7])
		sim.setJointTargetPosition(joint_3_HR,angles_joints[i+8])

		sim.setJointTargetPosition(joint_1_FR,angles_joints[i+9])
		sim.setJointTargetPosition(joint_2_FR,angles_joints[i+10])
		sim.setJointTargetPosition(joint_3_FR,angles_joints[i+11])
	end

	i = i+12

end

