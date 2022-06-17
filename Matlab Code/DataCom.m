function DataCom(topic, data)
   global state
   global length
   global measurements
   global measurementsIndex

   if state == 1
       state = 2;
       measurements = zeros(1, length);
   end
   if state == 2
       measurementsIndex = measurementsIndex + 1;
       measurements(measurementsIndex) = str2double(data);
   end
   if measurementsIndex == length
       state = 3;
       plotCallback
   end
end