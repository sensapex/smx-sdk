%% Compatible with 32 bit Matlab
%% 64 bit version did not work with Matlab 2007b

clc
clear all

loadlibrary('umanipulatorctl.dll','umanipulatorctl.h','alias','smx')

libfunctions  smx

libfunctionsview('smx')

[smx_handle, ret_string] = calllib('smx','umanipulatorctl_open','com7',uint32(200));

% Device ID is of "char" in C, which means that ascii to decimal conversion
% is required
[ret1, ret2] = calllib('smx','umanipulatorctl_select_dev',smx_handle,uint8(49))