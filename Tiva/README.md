Tiva Nodes
==================

Hopp3r is organized hierarchically, with peripherals like sensors and actuators connected to the master MCU over a CAN bus. Each node is controlled with a Tiva TM4C123GXL microcontroller. There are 3 categories of Tiva nodes:

1. Motor control node
2. Boom encoder node
3. Force sense and IMU node

Hopp3r has 3 motor control nodes, 3 boom encoder nodes, and 1 force sense/IMU node for a total of 7 nodes.

Here are the relevant files:

[Motor control node](/CAN_Copley/ver0)

[Boom encoder node](/CAN_Boom)

[Force sense/IMU node](/CAN_IMU_FZ/TxNode)
