set scriptpath=C:\Users\ejmcc\GIT Projects\CAN-Logger\src\hardware\Teensy\CANTransfer
FOR /f %%p in ('where python') do SET PYTHONPATH=%%p
%PYTHONPATH% "%scriptpath%\CANTransfer.py"
pause