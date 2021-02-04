# Brushless Motor Embedded System

On a real-time operating system (RTOS), control algorithms command a synchronous DC brushless motor using serial commands from user (through interrupts) whilst concurrently running a Bitcoin mining algorithm (as a low priority task) to demonstrate remaining CPU time after executing the motor control tasks.

The controls of the brushless motor are:
- angular velocity
- direction
- number of revolutions

**note: Crypto library imported from https://os.mbed.com/users/feb11/code/Crypto/
