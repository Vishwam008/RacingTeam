## How to solve utils not found
Apparently the python interpreter does not have utils in yolov5 in its path (and doesn't even look for it :/). Add the parent directory before utils in the import command in our case it is yolov5.

So if the line is `from utils import xyz` it shall be `from yolov5.utils import xyz`

