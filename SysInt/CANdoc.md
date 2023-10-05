# CAN- Controller Area Network

## Important: vcan is an interface of socketcan; they are not completely different things!!! :P
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
```
    bus = can.interface.Bus('node', bustype='virtual')
```

Then we shall create a message that can be passed onto the bus:

```
    msg = can.Message(arbitration_id=0xabcde, data=l)
    bus.send(msg)
```
where l is any type of data.
bus.send(msg) sends the message onto the bus.




## Util functions in terminal
```
$ sudo modprobe vcan
```
modprobe command adds or removes a module from the linux kernel.

```
$ sudo ip link add dev vcan0 type vcan
$ sudi ip link set up vcan
```
Added the device of type vcan named vcan0

```
cangen vcan0
```
generates and publishes random messages

```
cansend vcan0 1F334455#1122334455667788
```
Sends this specific message

```
$ candump vcan0
```
prints all the messages on vcan0
-l saves all these messages to a logfile while printing is disabled

```
$ cat candump-2023-09-10_000710.log | canplayer vcan0=vcan0
```
plays and publishes the messages stored on the logfile.

When using a real machine 'vcan' can be replaced by can and vcan0 by can0


## Encoding Decoding using python
```
>>> import cantools
>>> from pprint import pprint
>>> db = cantools.database.load_file('tests/files/dbc/motohawk.dbc')
>>> db.messages
[message('ExampleMessage', 0x1f0, False, 8, 'Example message used as template in MotoHawk models.')]
>>> example_message = db.get_message_by_name('ExampleMessage')
>>> pprint(example_message.signals)
```

### Encoding
```
>>> import can
>>> can_bus = can.interface.Bus('vcan0', bustype='socketcan')
>>> data = example_message.encode({'Temperature': 250.1, 'AverageRadius': 3.2, 'Enable': 1})
>>> message = can.Message(arbitration_id=example_message.frame_id, data=data)
>>> can_bus.send(message)
```

### Decoding
```
>>> message = can_bus.recv()
>>> db.decode_message(message.arbitration_id, message.data)
```

### To print the received message on the terminal
```
$ candump vcan0 | python3 -m cantools decode tests/files/dbc/motohawk.dbc
```


Each message has a list of signals which are the real value containers. (Check the interface datasheet)

When encoding a message, it shall be provided as a dictionary where EVERY signal is a key and all have thier respective data as values.