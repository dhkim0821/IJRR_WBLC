# Dynamic Controller

Each controller has its own interface, test, controller

Interaces connet target system and controller
For example when simulator has multiple system, the simulator call multiple interface lik
````
nao_interace->GetCommand(nao_sensor_data, nao_torque_cmd);
valkyrie_interface->GetCommand(valkyrie_sensor_data, valkyrie_torque_cmd);
````

Please refer 'NAO_Exam_Controller' to figure out the detail implementation method.
