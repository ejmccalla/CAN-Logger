set scriptpath=C:\Users\ejmcc\GIT Projects\CAN-Logger\src\hardware\Teensy\test
FOR /f %%p in ('where python') do SET PYTHONPATH=%%p
%PYTHONPATH% "%scriptpath%\test.py"
pause