% MIT License
%
% Copyright (c) 2020 Jasprit Singh Gill (jaspritsgill@gmail.com)
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

clear;

% Get path of this file, and add the parent folder and the subfolders into
% PATH
%   which('<filename>') returns the full path of the file
%   mfilename('fullpath') returns the full path of the file. Without any
%       arguments, this function returns just the filename
%   pwd returns the current working directory of matlab, which can be
%       different from the current file directory
fullFilename = mfilename('fullpath')
% get the parent folder from the full filepath
[filepath, name, ext] = fileparts(fullFilename);

% addpath('./motionModelClasses')
disp('Following files will be added to the path...');
for folder = ["/motionModelClasses", "/mmae_filters", "/filter_classes", "/VehicleParameters"]
    folder_path = append(filepath, folder);
    disp(folder_path)
    addpath(folder_path)
end
