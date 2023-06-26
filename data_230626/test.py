import matplotlib.pyplot as plt



def read_phases():
    file = open("dot_q.txt", "r")
    dist = file.readlines() # class str
    # phase_unwrap = []
    for line in dist:
        # phase_unwrap.append(float(line))
        print(line.split())



    # plt.plot(phase_unwrap)
    # plt.plot(distance)
    # plt.plot(np.unwrap(phase_unwrap))
    # plt.xlabel('X of tf_translation (m)')
    # plt.ylabel('Unwrapped Phase (rad)')
    # plt.title('data5')
    # plt.axis('equal')
    # plt.show()
    # plt.savefig('wrapped_phase.png')



if __name__ == '__main__':
    read_phases()