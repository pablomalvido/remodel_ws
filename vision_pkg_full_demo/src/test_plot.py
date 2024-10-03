import matplotlib.pyplot as plt
import os

fig, ax = plt.subplots()
ax.plot([0,1,16], [0,1,2], color='b')
ax.set_xlabel('x')
ax.set_ylabel('y')
#ax.set_aspect('equal')
ax.set_adjustable("datalim")
ax.set_ylim(0,2)
ax.set_title('Cables shape estimation')
ax.set_facecolor('violet')
plt.grid(True)
plt.show()