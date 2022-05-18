local function init_func()
	setSerialBaudrate(9600)
end

local function run_func()
	field = getFieldInfo('AccX')
	id = field.id
	-- xyz
	acc = getValue(id) .. ' ' .. getValue('AccY') .. ' ' .. getValue('AccZ') 
	-- rpy
	gyro = getValue('Roll') .. ' ' .. getValue('Pitc') .. ' ' .. getValue('Hdg')
	serialWrite( acc .. ' ' .. gyro .. '\n')
end

return { run=run_func, init=init_func }
