# the thrust of the drone varies as:
# F = k*rpm*(rpm-v) where v is the speed k is a constant
# We assume mass as 1kg
# thus net force on drone is F - mg

from matplotlib import pyplot as plt

curr_pos = 0
des_pos = 100

k1 = 0.1
k2 = 0.01
k3 = 0.1

c = 0 # counter for time
err = des_pos - curr_pos
vel = 0

p=8.345*(10**(-10))
q=4233*(10**(-4))
dt = 0.01

x = []
y = []

I = 0
perr = err # perr is previous error

while c < 100000:
    # print("here")
    P = k1*err  # Calculating the PID values
    I += k2*err*dt
    D = k3*(err - perr)/dt

    rpm = P + I + D # output of the PID controller
    acc = rpm*(abs(rpm-vel)) - 9.81 # performing kinematics
    vel = vel + acc*dt
    curr_pos += vel*dt

    perr = err
    err = des_pos - curr_pos

        
    x.append(c)
    y.append(curr_pos)
    c += 1

plt.plot(x, y)

plt.xlabel("Representation of time")
plt.ylabel("Altitude")
plt.show()
