function ConfigCom(topic, data)
    global state
    global length
    if state == 0 
        state = 1;
        length = str2double(data);
        % write(mqttClient, "ConfigCom", data);
    end
end