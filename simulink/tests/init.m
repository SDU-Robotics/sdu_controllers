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