function InstructionText(states,instruction)
if (instruction == 1)
    instructionString = 'Put your arms down';
    statusString = 'Control will begin in ';
    controlString1 = 'You are now controling the Robot!';
    fprintf([instructionString,' at your sides\n',statusString,'3...']);
    SetSimulationControlText(states,...
        instructionString,statusString,'','3...');
    pause(1); fprintf('2...'); SetSimulationControlText(states,...
        instructionString,statusString,'','3...2...');
    pause(1); fprintf('1...'); SetSimulationControlText(states,...
        instructionString,statusString,'','3...2...1...');
    pause(1); fprintf(['\n\n',controlString1,'\n\n']);
    SetSimulationControlText(states,'Interactive Simulation',...
        'Running Simulation...','Robot Arm Control','{Delete to quit}');
end