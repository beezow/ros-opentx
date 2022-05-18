local function init_func()
	setSerialBaudrate(9600)
end

local function run_func()
	field = getFieldInfo('AccX')
	id = field.id
	-- xyz
	acc = getValue(id) .. ' ' .. getValue('AccY') .. ' ' .. getValue('AccZ') 
	-- rpy
	gyro = getValue('g1') .. ' ' .. getValue('g2') .. ' ' .. getValue('g3')

	oron = getValue('Roll') .. ' ' .. getValue('Pitc') .. ' ' .. getValue('Hdg')
	
	serialWrite( acc .. ' ' .. oron .. ' ' .. gyro .. '\n')
end

return { run=run_func, init=init_func }
