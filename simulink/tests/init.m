clc
clear
close all

%%
pyenv(ExecutionMode="InProcess")

%%
% Add library path
addpath("../library/")

% Create robot model
sdu_controllers_lib = py.importlib.import_module('sdu_controllers')
bbrobot = sdu_controllers_lib.models.BreedingBlanketHandlingRobotModel();

ur3e = sdu_controllers_lib.models.URRobotModel( ...
    sdu_controllers_lib.models.RobotType(0));