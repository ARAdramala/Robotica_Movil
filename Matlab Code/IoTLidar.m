%% David Ramon Alaman - Robotica movil
% Proyecto Robot Movil
% IoT Lidar - MQTT

%% Connecting to MQTT Broker

% Load private data of the connection
load MQTT_Secrets.mat
% Stores the following variables
% brokerAddress, port, userName, password, clientID

% Begin connection
global mqttClient
mqttClient = mqttclient(brokerAddress, Port = port, ClientID = clientID,...
    Username = userName, Password = password);

%% Subscribe to Topics

% Configuration Topic
global length % Length of the incomming data
SubscrTable = subscribe(mqttClient, "ConfigCom", QualityOfService=2, Callback =@ConfigCom);

% Data Topic
global measurements % Array of measurements
global measurementsIndex % Index of the received data
measurementsIndex = 0;
SubscrTable = subscribe(mqttClient, "DataCom", QualityOfService=2, Callback =@DataCom);

%% State of the communication
% State:
% 0 = Waiting communication with the robot
% 1 = Data Length obtained
% 2 = Gathering Data
% 3 = Data obtained
% 4 = Plotting 

global state;
state = 0;