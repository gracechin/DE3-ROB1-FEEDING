import Franka
from time import sleep

print('here')

fred = Franka.Control()
fred.move_to(0.4, 0.4, 0.4, 0.1)
