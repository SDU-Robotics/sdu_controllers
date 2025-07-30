function blkStruct = slblocks
% This function specifies that the library 'sdu_controllers'
% should appear in the Library Browser with the 
% name 'SDU Controllers'

    Browser.Library = 'sdu_controllers';
    % 'sdu_controllers' is the name of the library

    Browser.Name = 'SDU Controllers';
    % 'SDU Controllers' is the library name that appears
    % in the Library Browser

    blkStruct.Browser = Browser;