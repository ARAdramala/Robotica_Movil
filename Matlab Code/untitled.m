function out = untitled(topic,data)
    global list
    disp(topic)
    class(data)
    str2num(data)
    list = [list, str2num(data)];
end