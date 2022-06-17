function testScript(len)
    clearvars -except len
    clc
    IoTLidar
    
    lens = num2str(len);
    
    write(mqttClient, "ConfigCom", lens);
    for i = 1:len
        pause(1)
        write(mqttClient, "DataCom", num2str(randi(500, 1)));
    end
end