from rover4ws_kinematics.src.modes.carLike import CarLike

if __name__ == '__main__':
    car = CarLike()
    car.kinematicsStep([1,0,-1])
    car.show()