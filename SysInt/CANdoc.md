# CAN- Controller Area Network
CAN is basically a bus that allows the sharing of information between multiple nodes.

all the nodes in a CAN are connected through a bus and a signal passed by any node is received by all other nodes

A node has the following main components:
<ol>
    <li>MCU- Microcontroller Unit</li>
    <li>CAN controller
        <ul>
            <li>CAN_Tx to transmit data</li>
            <li>CAN_Rx to receive data</li>
        </ul>
    <li>Transceiver</li>
</ol>

The bus consists of two channels: CAN H and CAN L for noise cancellation and two bus termination resistors

The signals are transmitted as differential signals, i.e. complementary pairs so subtracting them would cancel the noise

Logic 1: both CAN H and L are on same voltage so difference is 0 and the bus state is called recessive

Logic 0: both CAN L and H are on different voltage(H with higher) and the bus state is dominant

## Dominance
If two nodes try to access the bus at the same time the dominant signal always dominates:

Each node has an arbitration ID and to communicate each node puts the ID bitwise onto the bus. The node with lowest ID wins the arbitration and transmits the signal as (0 is dominant).

# Python-CAN
First, a virtual CAN interface needs to be set up:
<code>

    bus = can.interface.Bus('node', bustype='virtual')
</code>

Then we shall create a message that can be passed onto the bus:
<code>

    msg = can.Message(arbitration_id=0xabcde, data=l)
    bus.send(msg)
</code>
where l is any type of data.
bus.send(msg) sends the message onto the bus.

